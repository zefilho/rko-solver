#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>

#include "rkolib/core/problem.hpp"

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
    
    // Cada veículo é um objetivo a ser otimizado/balanceado
    int getNumObjectives() const override { return numVehicles; }
};

REGISTER_RKO_PROBLEM(VrpFairnessProblem)