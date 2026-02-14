#include "rkolib/mh/multistart.hpp"

// Dependências internas
#include "rkolib/core/method.hpp"  // Utils (CreateInitialSolutions, Decoder, etc.)
#include "rkolib/core/iproblem.hpp" // Definição de IProblem

namespace rkolib::mh {

    void MultiStart(const rkolib::core::TRunData &runData, const rkolib::core::IProblem &problem)
    {
        using namespace rkolib::core;

        const char* method = "MultiStart";
        
        TSol s;                         // current solution
        TSol sBest;                     // best solution

        int IterT = 0;                  // current iteration

        float currentTime = 0;          // computational time of the search process
        
        double start_timeMH = get_time_in_seconds();    // start computational time
        double end_timeMH = get_time_in_seconds();      // end computational time

        // Generate first solution
        CreateInitialSolutions(s, problem.getDimension()); 
        s.ofv = problem.evaluate(s);
        sBest = s;
        
        // run the search process until stop criterion
        while(currentTime < runData.MAXTIME * runData.restart)
        {
            if (SOLVER_SHOULD_STOP) return;      
            
            // Create a new solution with random keys 
            CreateInitialSolutions(s, problem.getDimension()); 
            s.ofv = problem.evaluate(s);

            // Verify improvement
            if (s.ofv < sBest.ofv)
            {
                sBest = s;

                // update the SOLVER_POOL of solutions
                UpdatePoolSolutions(sBest, method, runData.debug);
            }

            IterT++;

            // terminate the evolutionary process in MAXTIME
            end_timeMH = get_time_in_seconds();
            currentTime = (float)(end_timeMH - start_timeMH);
        }
    }

} // namespace rkolib::mh