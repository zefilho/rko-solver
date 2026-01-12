#pragma once

#include "rkolib/core/data.hpp" // Para TSol e Common

// Forward declaration: Avisa que TProblemData existe, sem precisar do include pesado aqui.
// (Assumindo que TProblemData foi definido no seu KnapsackProblem.hpp ou similar)
struct TProblemData;

namespace rkolib::utils {

    /**
     * Outputs the solution to the screen using the Decoder.
     */
    void WriteSolutionScreen(const char *algorithms[], int numMH, rkolib::core::TSol s, 
                             float timeBest, float timeTotal, char instance[], 
                             const TProblemData &data, std::vector<rkolib::core::TSol> pool);

    /**
     * Outputs the solution in a txt file.
     * Note: Requires "../Results/" folder to exist.
     */
    void WriteSolution(const char *algorithms[], int numMH, rkolib::core::TSol s, 
                       float timeBest, float timeTotal, char instance[], 
                       const TProblemData &data);

    /**
     * Outputs the results in a csv file.
     * Note: Requires "../Results/" folder to exist.
     */
    void WriteResults(const char *algorithms[], int numMH, double ofv, 
                      double ofvAverage, std::vector<double> ofvs, float timeBest, 
                      float timeTotal, char instance[]);

} // namespace rkolib::utils