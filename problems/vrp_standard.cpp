#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>

#include "rkolib/core/problem.hpp"

class VrpStandardProblem : public rkolib::core::IProblem {
private:
    int numNodes;     
    int numVehicles;  
    std::vector<std::vector<double>> distMatrix;

public:
    VrpStandardProblem() : numNodes(0), numVehicles(0) {}
    ~VrpStandardProblem() override = default;

    void load(const std::string &name) override {
        std::ifstream file(name);
        if (!file.is_open()) throw std::runtime_error("File not found: " + name);

        std::string line;
        std::getline(file, line);
        std::stringstream ss(line);
        ss >> numNodes >> numVehicles;

        std::cout << "[VRP Standard] Loaded: " << numNodes - 1 << " clients, " 
                  << numVehicles << " vehicles.\n";

        distMatrix.assign(numNodes, std::vector<double>(numNodes, 0.0));
        for (int i = 0; i < numNodes; i++) {
            for (int j = 0; j < numNodes; j++) {
                file >> distMatrix[i][j];
            }
        }
    }

    void decode(rkolib::core::TSol &s) const override {
        int numClients = numNodes - 1;

        // 1. SEQUENCE-FIRST: Rota Gigante
        std::vector<std::pair<double, int>> giantTour(numClients);
        for (int i = 0; i < numClients; ++i) {
            giantTour[i] = {s.rk[i], i + 1}; 
        }
        std::sort(giantTour.begin(), giantTour.end());

        // 2. SPLIT-SECOND: Cálculo da rota
        double totalDistance = 0.0; // <-- A MUDANÇA ESTÁ AQUI
        int clientsPerVehicle = std::ceil((double)numClients / numVehicles);
        int currentClientIdx = 0;

        for (int v = 0; v < numVehicles; ++v) {
            int currentLocation = 0; 
            
            for (int k = 0; k < clientsPerVehicle && currentClientIdx < numClients; ++k) {
                int nextLocation = giantTour[currentClientIdx].second;
                
                totalDistance += distMatrix[currentLocation][nextLocation];
                currentLocation = nextLocation;
                
                currentClientIdx++;
            }
            
            totalDistance += distMatrix[currentLocation][0];
        }

        // 3. RETORNO MONO-OBJETIVO:
        // Ignoramos a individualidade dos motoristas. O Fitness é o custo total da operação.
        s.ofv = totalDistance; 
        
        // Limpamos o array de objetivos secundários para garantir que o Solver 
        // entenda que é um problema puramente Mono-objetivo.
        s.objs.clear(); 
    }

    int getDimension() const override { return numNodes - 1; }
    
    // <-- O Solver agora sabe que só existe 1 objetivo (Custo Total)
    int getNumObjectives() const override { return 1; } 
};

REGISTER_RKO_PROBLEM(VrpStandardProblem)