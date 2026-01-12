#include "rkolib/mh/ipr.hpp"

// Dependências internas
#include "rkolib/core/method.hpp"         // irandomico, pool, get_time, UpdatePoolSolutions
#include "rkolib/core/problem.hpp" // Para data.n

namespace rkolib::mh {

    void IPR(const rkolib::core::TRunData &runData, const rkolib::core::TProblemData &data)
    {
        using namespace rkolib::core;

        const char* method = "IPR";

        float currentTime = 0;                          // computational time of the search process
        double start_timeMH = get_time_in_seconds();    // start computational time
        double end_timeMH = get_time_in_seconds();      // end computational time

        // Variáveis que persistem durante o processo (como o melhor caminho encontrado globalmente)
        // Nota: Removi declarações redundantes de 'atual', 'guia', etc., que eram sombreadas no loop.
        
        // run the search process until stop criterion
        while (currentTime < runData.MAXTIME * runData.restart)
        {
            // Verifica se o pool tem soluções suficientes
            if (pool.size() < 2) break;

            // randomly choose two elite solutions
            int k1, k2;
            do {
                k1 = irandomico(0, (int)pool.size() - 1);
                k2 = irandomico(0, (int)pool.size() - 1);
            }
            while (k1 == k2);

            // Escopo do loop: variáveis reinicializadas a cada iteração do while
            TSol atual = pool[k1];                       
            TSol guia = pool[k2];                        

            TSol bestPath = atual;                  
            TSol bestIteration = atual;                 
            TSol sCurrent = atual;                      
            TSol sViz = atual;                          

            int direction = 1;                          // internal (1) or external (-1) IPR

            // Correção de Senior: Garantir que blockSize nunca seja 0
            int blockSize = std::max(1, (int)(data.n * 0.10)); 
            int numBlock = data.n / blockSize;             
            
            std::vector<int> fixedBlock(numBlock, 1);   // binary vector: 1 = not swapped, 0 = swap candidate
            int dist = 0;                               // number of different rk blocks

            // calculates the difference between the solutions (measured by the number of different blocks)
            for (int i = 0; i < data.n; i++)
            {
                // Comparação segura de double
                if (std::abs(atual.rk[i] - guia.rk[i]) > 1e-9){
                    // Proteção de índice do bloco
                    int blockIdx = i % numBlock;
                    if (fixedBlock[blockIdx] != 0) {
                        fixedBlock[blockIdx] = 0; // Marca o bloco como diferente
                        dist++; // Incrementa contagem de blocos diferentes (simplificado)
                    }
                }
            }

            // if there is a difference between the solutions 
            if (dist > 0)
            {
                // 'zero out' the best solution on the path and carry out the search
                bestPath.ofv = std::numeric_limits<double>::infinity();
                int bestBlock = -1;

                // continue as long as there are blocks to exchange
                int numIteration = 0;
                
                // Nota: O loop original limitava em numBlock-1, mantido.
                while(numIteration < numBlock - 1)
                {
                    if (stop_execution.load()) return;

                    numIteration++;
                    bestBlock = -1;
                    bestIteration.ofv = std::numeric_limits<double>::infinity();

                    // examine all possible blocks in a IPR iteration
                    for(int i = 0; i < numBlock; i++)
                    {
                        if (fixedBlock[i] == 0){
                            int initialBlock = i * blockSize;
                            int finalBlock = initialBlock + blockSize;

                            // generate a neighbor of the current solution
                            sViz = sCurrent;
                            for (int k = initialBlock; k < finalBlock && k < data.n; k++)
                            {
                                // internal PR
                                if (direction == 1){
                                    sViz.rk[k] = guia.rk[k];
                                }
                                
                                // external PR
                                else if (direction == -1){
                                    sViz.rk[k] = std::max(0.0, std::min(1.0 - guia.rk[k], 0.999999));
                                }
                            }
                            
                            sViz.ofv = Decoder(sViz, data);

                            // check if it is the best solution of the iteration
                            if(sViz.ofv < bestIteration.ofv){
                                bestIteration = sViz;
                                bestBlock = i;
                            }
                        }
                    }

                    // check if it is the best solution for the path
                    if (bestIteration.ofv < bestPath.ofv)
                    {
                        bestPath = bestIteration;
                    }
                    
                    if (bestBlock >= 0){
                        // continue the search from the best solution of the iteration
                        sCurrent = bestIteration;

                        // fixed the index of the block changed in this iteration
                        fixedBlock[bestBlock] = 1;
                    } else {
                        // Se nenhum bloco melhorou ou foi encontrado, para a busca
                        break;
                    }
                }
            }

            // terminate the search process in MAXTIME
            end_timeMH = get_time_in_seconds();
            currentTime = (float)(end_timeMH - start_timeMH);

            // update the pool of solutions
            UpdatePoolSolutions(bestPath, method, runData.debug);
        }
    }

} // namespace rkolib::mh