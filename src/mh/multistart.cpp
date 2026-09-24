#include "rkolib/mh/multistart.hpp"

#include "rkolib/core/method.hpp"
#include "rkolib/core/solver.hpp"

namespace rkolib::mh {

void MultiStart(const rkolib::core::TRunData &runData,
                rkolib::RkoSolver &solver) {
  using namespace rkolib::core;

  const char *method = "MultiStart";

  TSol s;     // current solution
  TSol sBest; // best solution

  int IterT = 0; // current iteration

  float currentTime = 0; // computational time of the search process

  double start_timeMH = get_time_in_seconds(); // start computational time
  double end_timeMH = get_time_in_seconds();   // end computational time

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] MultiStart iniciado." << std::endl;
  }

  // Generate first solution
  CreateInitialSolutions(s, solver.getProblemDimension());
  solver.decodeSolution(s);
  sBest = s;

  // run the search process until stop criterion
  while (currentTime < runData.MAXTIME * runData.restart) {
    if (SOLVER_SHOULD_STOP) {
      if (runData.debug > 0) {
        std::cout << "[DEBUG][MH] MultiStart interrompido via SOLVER_SHOULD_STOP." << std::endl;
      }
      return;
    }

    // Create a new solution with random keys
    CreateInitialSolutions(s, solver.getProblemDimension());
    solver.decodeSolution(s);

    // Verify improvement
    if (s.ofv < sBest.ofv) {
      sBest = s;

      // update the SOLVER_POOL of solutions
      UpdatePoolSolutions(sBest, method, runData.debug, runData.poolUpdateMethod);
    }

    IterT++;

    // terminate the evolutionary process in MAXTIME
    end_timeMH = get_time_in_seconds();
    currentTime = (float)(end_timeMH - start_timeMH);
  }

  if (runData.debug > 0) {
    std::cout << "[DEBUG][MH] MultiStart finalizado. Iteracoes=" << IterT << ", Melhor OFV=" << sBest.ofv << std::endl;
  }
}

} // namespace rkolib::mh