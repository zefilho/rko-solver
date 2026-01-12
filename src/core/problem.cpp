#include "rkolib/core/data.hpp"
#include "rkolib/core/problem.hpp"

//-------------------------- IMPLEMENTATION --------------------------
namespace rkolib::core {

    void ReadData(const char* name, TProblemData &data)
    { 
        FILE *arq;
        arq = fopen(name, "r");

        if (arq == NULL)
        {
            printf("\nERROR: File (%s) not found!\n", name);
            std::cin.get();
            exit(1);
        }

        // => read data
        fscanf(arq, "%d", &data.nItems);
        fscanf(arq, "%d", &data.cap);
        
        // weigth of items
        data.w.clear();
        data.w.resize(data.nItems);

        // prize of items
        data.b.clear();
        data.b.resize(data.nItems);

        for (int k = 0; k < data.nItems; k++)
        {
            fscanf(arq, "%d", &data.b[k]);
            fscanf(arq, "%d", &data.w[k]);
        }
        
        fclose(arq); // Fechando arquivo após abrir

        // define the random-key vector size
        data.n = data.nItems;
    }

    double Decoder(const TSol &s, const TProblemData &data)
    {   
        // create a solution of the KP
        std::vector<int> sol(data.n, 0);                         
        for (int i = 0; i < data.n; i++)
        {
            // Nota: Assumindo que s.rk é acessível.
            if (s.rk[i] > 0.5)
                sol[i] = 1;
        }

        // calculate the objective function value
        int cost = 0;
        int totalW = 0;
        for (int i = 0; i < data.n; i++)
        {
            if (sol[i] == 1)
            {
                cost += data.b[i];
                totalW += data.w[i];
            }
        }

        // penalty infeasible solutions
        // Substituída a macro MAX por std::max e lógica simplificada
        int infeasible = std::max(0, totalW - data.cap);
        
        // Convertendo para double para cálculo preciso antes de retornar
        double finalCost = (double)cost - (100000.0 * infeasible);

        // change to minimization problem
        return finalCost * -1.0;
    }

    void FreeMemoryProblem(TProblemData &data){
        // Em C++ moderno, vectors se limpam sozinhos quando saem de escopo.
        // Mas se quiser forçar a limpeza para reutilizar o objeto data:
        data.b.clear();
        data.w.clear();
    }

}