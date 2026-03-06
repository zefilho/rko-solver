#pragma once
#include "rkolib/core/data.hpp"

namespace rkolib::core {

// Abstract Interface
class IProblem {
public:
  virtual ~IProblem() = default;

  // Loads data from file
  virtual void load(const std::string &filename) = 0;

  // Calculates raw objectives and populates s.objs[]
  //  Does NOT apply weights here (that's Solver's job)
  virtual void decode(TSol &s) const = 0;

  // Getters essenciais
  virtual int getDimension() const = 0;     // Number of variables
  virtual int getNumObjectives() const = 0; // 1 = Mono, >1 = Multi
};

// // Declaration of a "factory" that user will implement
// std::shared_ptr<IProblem> createProblem();

} // namespace rkolib::core

extern "C" {
// Type of the function that the Solver will call to create the problem
typedef rkolib::core::IProblem *(*CreateProblemFunc)();

// Type of the function that the Solver will call to destroy the problem (avoids
// memory leaks)
typedef void (*DestroyProblemFunc)(rkolib::core::IProblem *);
}

// Cross-platform macros to export the function in the problem file
#ifdef _WIN32
#define EXPORT_PLUGIN __declspec(dllexport)
#else
#define EXPORT_PLUGIN __attribute__((visibility("default")))
#endif