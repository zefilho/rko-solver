#include "rkolib/mh/multistart.hpp"

// Dependências internas
#include "rkolib/core/method.hpp"  // Utils (CreateInitialSolutions, Decoder, etc.)
#include "rkolib/core/problem.hpp" // Definição de TProblemData

namespace rkolib::mh {

    void MultiStart(const rkolib::core::TRunData &runData, const rkolib::core::TProblemData &data)
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
        CreateInitialSolutions(s, data.n); 
        s.ofv = Decoder(s, data);
        sBest = s;
        
        // run the search process until stop criterion
        while(currentTime < runData.MAXTIME * runData.restart)
        {
            if (stop_execution.load()) return;      
            
            // Create a new solution with random keys 
            CreateInitialSolutions(s, data.n); 
            s.ofv = Decoder(s, data);

            // Verify improvement
            if (s.ofv < sBest.ofv)
            {
                sBest = s;

                // update the pool of solutions
                UpdatePoolSolutions(sBest, method, runData.debug);
            }

            IterT++;

            // terminate the evolutionary process in MAXTIME
            end_timeMH = get_time_in_seconds();
            currentTime = (float)(end_timeMH - start_timeMH);
        }
    }

} // namespace rkolib::mh