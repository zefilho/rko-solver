#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

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

class TspProblem : public rkolib::core::IProblem {
private:
  int nCities, nObj;

  // 3D Matrix: objectives[k][i][j] = cost of objective 'k' going from city 'i'
  // to 'j'
  std::vector<std::vector<std::vector<double>>> costMatrices;

public:
  TspProblem() : nCities(0), nObj(1) {}

  ~TspProblem() override = default;

  void load(const std::string &name) override {
    std::ifstream file(name);
    if (!file.is_open())
      throw std::runtime_error("File not found: " + name);

    // 1. Reading the header
    std::string line;
    std::getline(file, line);
    std::stringstream ss(line);

    ss >> nCities;

    // Try to read the number of objectives. If it fails, it's mono-objective (1).
    if (!(ss >> nObj)) {
      nObj = 1;
    }

    std::cout << "[DEBUG] Loading TSP: nCities=" << nCities << ", nObj=" << nObj
              << std::endl;

    // Resize the structure to support 'nObj' matrices of size
    // 'nCities x nCities'
    costMatrices.assign(nObj, std::vector<std::vector<double>>(
                                  nCities, std::vector<double>(nCities, 0.0)));

    // 2. Reading the body (Reads each matrix in sequence)
    for (int k = 0; k < nObj; k++) {
      if (nObj > 1) {
        std::cout << "[DEBUG] Lendo matriz para o Objetivo " << k + 1
                  << std::endl;
      }

      for (int i = 0; i < nCities; i++) {
        for (int j = 0; j < nCities; j++) {
          file >> costMatrices[k][i][j];
        }
      }
    }
  }

  void decode(rkolib::core::TSol &s) const override {
    // 1. Create pairs (RandomKey, OriginalIndex) to generate the tour
    std::vector<std::pair<double, int>> tour(nCities);
    for (int i = 0; i < nCities; ++i) {
      tour[i] = {s.rk[i], i};
    }

    // 2. Sort ascending: The order of the keys defines the order of visitation of
    // the cities
    std::sort(tour.begin(), tour.end());

    // Zero/Resize the objectives in the solution
    if (s.objs.size() != (size_t)nObj)
      s.objs.assign(nObj, 0.0);
    else
      std::fill(s.objs.begin(), s.objs.end(), 0.0);

    // 3. Decoder: Calculates the cost of the generated tour
    for (int i = 0; i < nCities - 1; ++i) {
      int from = tour[i].second;
      int to = tour[i + 1].second;

      // Sum the costs for all objectives
      for (int k = 0; k < nObj; k++) {
        s.objs[k] += costMatrices[k][from][to];
      }
    }

    // 4. Finalization: Return to the origin city to close the cycle
    int lastNode = tour[nCities - 1].second;
    int firstNode = tour[0].second;

    for (int k = 0; k < nObj; k++) {
      s.objs[k] += costMatrices[k][lastNode][firstNode];
    }

    // 5. Fitness Definition (OFV)
    if (nObj == 1) {
      // TSP is naturally a MINIMIZATION problem.
      // Unlike the knapsack (profit), here the smaller the distance, the better.
      // Therefore, we assign the value directly, without negating.
      s.ofv = s.objs[0];
    }

    // NOTE: If nObj > 1, we leave s.ofv as 0.0 here.
    // The RkoSolver will calculate the Tchebycheff distance in the central engine.
  }

  int getDimension() const override { return nCities; }
  int getNumObjectives() const override { return nObj; }
};

// ---------------------------------------------------------
// EXPORTAÇÃO DO PLUGIN (Contrato C)
// ---------------------------------------------------------
extern "C" {

  #ifdef _WIN32
    __declspec(dllexport) rkolib::core::IProblem *create_problem() {
      return new TspProblem();
    }
    __declspec(dllexport) void destroy_problem(rkolib::core::IProblem *p) {
      delete p;
    }
  #else
    __attribute__((visibility("default"))) rkolib::core::IProblem *
    create_problem() {
      return new TspProblem();
    }
    __attribute__((visibility("default"))) void
    destroy_problem(rkolib::core::IProblem *p) {
      delete p;
    }
  #endif
}