

#include "rkolib/core/data.hpp"
#include "rkolib/core/problem.hpp"

//-------------------------- IMPLEMENTATION --------------------------
namespace rkolib::core {

    void ReadData(const std::string& name, TProblemData &data)
    { 
        // Correção 1 & 4: Usando std::ifstream em vez de fopen/fscanf (Seguro e Moderno)
        // Isso elimina os avisos de depreciação do _CRT_SECURE_NO_WARNINGS
        std::ifstream file(name);

        // Correção 2: nullptr literal (embora ifstream use checks booleanos)
        if (!file.is_open())
        {
            std::cout << "\nERROR: File (" << name << ") not found!" << std::endl ;
            std::cin.get();
            exit(1);
        }

        // Leitura formatada segura
        if (!(file >> data.nItems >> data.cap)) {
            std::cout << "ERROR: Failed to read header data." << std::endl;
            exit(1);
        }
        
        data.w.assign(data.nItems, 0);
        data.b.assign(data.nItems, 0);

        for (int k = 0; k < data.nItems; k++)
        {
            if (!(file >> data.b[k] >> data.w[k])) {
                break; 
            }
        }
        
        // O arquivo fecha automaticamente ao sair do escopo
        data.n = data.nItems;
    }

    double Decoder(const TSol &s, const TProblemData &data)
    {   
        // calculate the objective function value
        int cost = 0;
        int totalW = 0;
        for (int i = 0; i < data.n; i++)
        {
            if (s.rk[i] > 0.5)
            {
                cost += data.b[i];
                totalW += data.w[i];
            }
        }

        // penalty infeasible solutions
        // Substituída a macro MAX por std::max e lógica simplificada
        int infeasible = std::max(0, totalW - data.cap);
        
        // Convertendo para double para cálculo preciso antes de retornar
        double finalCost = static_cast<double>(cost) - (100000.0 * infeasible);

        // change to minimization problem
        return finalCost * -1.0;
    }

    void FreeMemoryProblem(TProblemData &data){
        // Em C++ moderno, vectors se limpam sozinhos quando saem de escopo.
        // Mas se quiser forçar a limpeza para reutilizar o objeto data:
        // data.b.clear();
        // data.w.clear();
        data.b.clear();
        data.b.shrink_to_fit();
        data.w.clear();
        data.w.shrink_to_fit();
    }

}