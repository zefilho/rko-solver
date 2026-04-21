#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>


namespace rkolib::core {

struct TSol {
  std::vector<double> rk;
  double ofv = std::numeric_limits<double>::infinity();
  double best_time = 0.0;
  std::string nameMH;
  std::vector<double> objs;
  TSol() = default;
};

// The virtual functions must be in exactly the same order as in your problem.hpp file.
class IProblem {
public:
  virtual ~IProblem() = default;
  virtual void load(const std::string &filename) = 0;
  virtual void decode(TSol &sol) const = 0;
  virtual int getDimension() const = 0;
  virtual int getNumObjectives() const = 0;
};
} // namespace rkolib::core

class VrpFairnessProblem : public rkolib::core::IProblem {
private:
    int numNodes;     // Depósito + Clientes
    int numVehicles;  // Quantidade de Motoristas (Stakeholders)
    std::vector<std::vector<double>> distMatrix;

public:
    VrpFairnessProblem() : numNodes(0), numVehicles(0) {}
    ~VrpFairnessProblem() override = default;

    void load(const std::string &name) override {
        std::ifstream file(name);
        if (!file.is_open()) throw std::runtime_error("File not found: " + name);

        // 1. Lendo cabeçalho
        std::string line;
        std::getline(file, line);
        std::stringstream ss(line);
        ss >> numNodes >> numVehicles;

        std::cout << "[VRP Plugin] Loaded: " << numNodes - 1 << " clients, " 
                  << numVehicles << " vehicles.\n";

        // 2. Lendo matriz de distância
        distMatrix.assign(numNodes, std::vector<double>(numNodes, 0.0));
        for (int i = 0; i < numNodes; i++) {
            for (int j = 0; j < numNodes; j++) {
                file >> distMatrix[i][j];
            }
        }
    }

    void decode(rkolib::core::TSol &s) const override {
        int numClients = numNodes - 1; // O nó 0 é o depósito

        // 1. SEQUENCE-FIRST: Decodifica o vetor RK em uma rota gigante (Giant Tour)
        std::vector<std::pair<double, int>> giantTour(numClients);
        for (int i = 0; i < numClients; ++i) {
            giantTour[i] = {s.rk[i], i + 1}; // +1 porque o cliente 0 na verdade é o nó 1 na matriz
        }
        std::sort(giantTour.begin(), giantTour.end()); // Ordem crescente dos RKs

        // Prepara o vetor de objetivos (Distância de CADA motorista)
        s.objs.assign(numVehicles, 0.0);

        // 2. SPLIT-SECOND: Divide a rota gigante entre os veículos de forma contígua
        int clientsPerVehicle = std::ceil((double)numClients / numVehicles);
        int currentClientIdx = 0;

        for (int v = 0; v < numVehicles; ++v) {
            int currentLocation = 0; // Todos saem do depósito (nó 0)
            double routeDistance = 0.0;

            // Atribui uma fatia de clientes a este veículo
            for (int k = 0; k < clientsPerVehicle && currentClientIdx < numClients; ++k) {
                int nextLocation = giantTour[currentClientIdx].second;
                
                // Soma a distância da localização atual para a próxima
                routeDistance += distMatrix[currentLocation][nextLocation];
                currentLocation = nextLocation;
                
                currentClientIdx++;
            }

            // Motorista volta para o depósito ao fim do dia
            routeDistance += distMatrix[currentLocation][0];
            
            // Salva a distância total DESSE motorista como um objetivo independente
            s.objs[v] = routeDistance;
        }

        // Deixamos s.ofv vazio (0 ou Infinity). 
        // O IScalarizer (Gini, Jain ou Tradeoff) vai ler s.objs e calcular o Fitness (s.ofv).
    }

    // A dimensão do problema é apenas o número de clientes que precisamos ordenar
    int getDimension() const override { return numNodes - 1; }
    
    // O pulo do gato da Justiça: Cada veículo é um objetivo a ser otimizado/balanceado
    int getNumObjectives() const override { return numVehicles; }
};

// ---------------------------------------------------------
// EXPORTAÇÃO DO PLUGIN (Contrato C)
// ---------------------------------------------------------
extern "C" {
    #ifdef _WIN32
        __declspec(dllexport) rkolib::core::IProblem *create_problem() {
            return new VrpFairnessProblem();
        }
        __declspec(dllexport) void destroy_problem(rkolib::core::IProblem *p) {
            delete p;
        }
    #else
        __attribute__((visibility("default"))) rkolib::core::IProblem *create_problem() {
            return new VrpFairnessProblem();
        }
        __attribute__((visibility("default"))) void destroy_problem(rkolib::core::IProblem *p) {
            delete p;
        }
    #endif
}