#pragma once // Padrão moderno para evitar inclusão múltipla

namespace rkolib::core {
    //----------------- DEFINITION OF PROBLEM SPECIFIC TYPES -----------------------
    struct TProblemData
    {
        int n;                          // size of the RKO vector 
        int nItems;                     // number of items
        int cap;                        // capacity of the knapsack
        std::vector<int> w;             // weigth of the items
        std::vector<int> b;             // prize of the items
    };

    //-------------------------- FUNCTIONS OF SPECIFIC PROBLEM --------------------------

    /**
     * Method: ReadData
     * Description: read the input data from a file
     */ 
    void ReadData(const char* name, TProblemData &data);

    /**
     * Method: Decoder 
     * Description: mapping the random-key solution into a problem solution
     */
    double Decoder(const TSol &s, const TProblemData &data);

    /**
     * Method: FreeMemoryProblem
     * Description: Free local memory allocate by Problem
     */
    void FreeMemoryProblem(TProblemData &data);
}


