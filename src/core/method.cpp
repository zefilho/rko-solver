#include "rkolib/core/method.hpp"

#include <omp.h>     // OpenMP
#include <yaml-cpp/yaml.h>

namespace rkolib::core {

    // -----------------------------------------------------------------------------
    // General Utilities
    // -----------------------------------------------------------------------------

    bool sortByFitness(const TSol &lhs, const TSol &rhs) { 
        return lhs.ofv < rhs.ofv; 
    }

    double randomico(double min, double max)
    {
        // Usa a variável global 'rng' declarada como extern no .hpp
        return std::uniform_real_distribution<double>(min, max)(rng);
    }

    int irandomico(int min, int max)
    {
        return (int)randomico(0, max - min + 1) + min;
    }

    double get_time_in_seconds() {
        #if defined(_WIN32) || defined(_WIN64)
            LARGE_INTEGER frequency, timeCur;
            QueryPerformanceFrequency(&frequency);
            QueryPerformanceCounter(&timeCur);
            return static_cast<double>(timeCur.QuadPart) / frequency.QuadPart;
        #else
            struct timespec timeCur;
            clock_gettime(CLOCK_MONOTONIC, &timeCur);
            return timeCur.tv_sec + timeCur.tv_nsec / 1e9;
        #endif
    }

    // -----------------------------------------------------------------------------
    // Solution & Pool Management
    // -----------------------------------------------------------------------------

    void CreateInitialSolutions(TSol &s, const int n)
    {
        s.rk.resize(n);
        // create a random-key solution
        for (int j = 0; j < n; j++){
            s.rk[j] = randomico(0, 1);  // random value between [0,1)
        }
    }

    void CreatePoolSolutions(const TProblemData &data, const int sizePool)
    {
        // Nota: Acessa a variável global 'pool'
        #pragma omp critical
        {
            for (int i = 0; i < sizePool; i++){
                CreateInitialSolutions(pool[i], data.n);
                pool[i].ofv = Decoder(pool[i], data); // Chama Decoder externo
                pool[i].best_time = get_time_in_seconds();
            }

            // sort pool in increasing order of fitness
            std::sort(pool.begin(), pool.begin() + sizePool, sortByFitness);

            // verify if similar solutions exist in the pool
            int clone = 0;
            // Cuidado: loop reverso original mantido, mas verifique se sizePool <= pool.size()
            for (int i = sizePool - 1; i > 0; i--){
                if (std::abs(pool[i].ofv - pool[i-1].ofv) < 1e-9){ // Comparação double segura
                    for (int j = 0; j < 0.2 * data.n; j++){
                        int pos = irandomico(0, data.n - 1);
                        pool[i].rk[pos] = randomico(0, 1);
                    }
                    pool[i].ofv = Decoder(pool[i], data);
                    clone = 1;
                }
            }

            // sort pool again if clones were mutated
            if (clone)
            {
                std::sort(pool.begin(), pool.begin() + sizePool, sortByFitness);
            }
        }
    }

    void UpdatePoolSolutions(TSol s, const char* mh, const int debug)
    {
        #pragma omp critical
        {   
            // Checks if it already exists in the pool
            bool exists = false;
            for (int i = 0; i < (int)pool.size(); i++) {
                if (std::abs(pool[i].ofv - s.ofv) < 1e-9) { // Comparação segura
                    exists = true;
                    break;
                }
            }

            // print that a new best solution was found
            if (!pool.empty() && s.ofv < pool[0].ofv && debug) 
            {
                int thread_id = omp_get_thread_num();   
                printf("\nBest solution: %.10lf (Thread: %d - MH: %s)", s.ofv, thread_id, mh);
            } 

            // Goes from back to front
            if (!exists)
            {
                s.best_time = get_time_in_seconds();
                s.nameMH = mh; // std::string assignment

                // insert the new solution preserving order (Insertion Sort logic)
                int i;
                for (i = (int)pool.size() - 1; i > 0 && pool[i - 1].ofv > s.ofv; i--) {
                    pool[i] = pool[i - 1]; // Push to the right
                }
                pool[i] = s; 
            }
        }
    }

    // -----------------------------------------------------------------------------
    // Local Searches & Components
    // -----------------------------------------------------------------------------

    void ShakeSolution(TSol &s, float betaMin, float betaMax, const int n)
    {
        int intensity = (int)(n * randomico(betaMin, betaMax)) + 1;
        if (intensity < 1) intensity = 1;
        
        for(int k = 0; k < intensity; k++) {
            int shaking_type = irandomico(1, 4);
            int i = irandomico(0, n - 1);
            
            if(shaking_type == 1){
                s.rk[i] = randomico(0, 1);
            }
            else if(shaking_type == 2){
                if (s.rk[i] > 0.0001) s.rk[i] = 1.0 - s.rk[i];
                else s.rk[i] = 0.9999;
            }
            else if (shaking_type == 3){
                int j = irandomico(0, n - 1);
                std::swap(s.rk[i], s.rk[j]);
            }
            
            if(shaking_type == 4 && n > 1){
                // Swap with neighbor (cuidado com limites)
                int idx = irandomico(0, n - 2); 
                std::swap(s.rk[idx], s.rk[idx+1]);
            }
        }
    }

    TSol Blending(TSol &s1, TSol &s2, double factor, const int n)
    {   
        TSol s;
        s.rk.resize(n);

        for(int j = 0; j < n; j++)
        {
            double value;
            if (randomico(0, 1) < 0.02)
            { // mutation
                value = randomico(0, 1);
            } else if (randomico(0, 1) < 0.5) {
                value = s1.rk[j];
                continue;
            } else if (factor == -1) { // Nelder-Mead reflection logic implicitly
                value = std::clamp(1.0 - s2.rk[j], 0.0, 0.9999999);
            }
            else {
                value = s2.rk[j];
            }
            s.rk[j] = value;
        }
        return s;
    }

    void NelderMeadSearch(TSol &x1, const TProblemData &data)
    {
        int poolSize = (int)pool.size();
        if (poolSize < 2) return; // Segurança

        TSol x1Origem = x1;
        TSol xBest = x1;

        bool improved = 0;
        bool improvedX1 = 0; //Rastreia se x1 foi melhorado
        
        // Helper: Avalia uma solução candidata e atualiza o xBest se necessário
        auto evaluateAndUpdate = [&](TSol& candidate) {
            candidate.ofv = Decoder(candidate, data);
            if (candidate.ofv < xBest.ofv) {
                xBest = candidate;
                improved = true;
                improvedX1 = true;
            }
        };

        auto sortSimplex = [](TSol& a, TSol& b, TSol& c) {
            if (a.ofv > b.ofv) std::swap(a, b);
            if (a.ofv > c.ofv) std::swap(a, c);
            if (b.ofv > c.ofv) std::swap(b, c);
        };
        
        //Selecao de x2 e x3 aleatório
        int k1, k2;
        do {
            k1 = irandomico(0, poolSize - 1);
            k2 = irandomico(0, poolSize - 1);
        } while (k1 == k2);

        TSol x2 = pool[k1];
        TSol x3 = pool[k2];

        // Sort x1, x2, x3
        sortSimplex(x1, x2, x3);
        
        // Centroid
        TSol x0 = Blending(x1, x2, 1, data.n);
        evaluateAndUpdate(x0);

        int iter_count = 1; 
        int maxIter = std::max(10, static_cast<int>(data.n * std::exp(-2)));

        while (iter_count <= maxIter) 
        {
            bool shrink = false;

            // Reflection
            TSol x_r = Blending(x0, x3, -1, data.n);
            evaluateAndUpdate(x_r);

            if (x_r.ofv < x1.ofv) 
            {
                // Expansion
                TSol x_e = Blending(x_r, x0, -1, data.n);
                evaluateAndUpdate(x_e);
                x3 = (x_e.ofv < x_r.ofv) ? x_e : x_r;
            } 
            else if (x_r.ofv < x2.ofv) {
                x3 = x_r;
            } 
            else {
                bool outsite = x_r.ofv < x3.ofv;
                TSol x_c = outsite ? Blending(x_r, x0, 1, data.n) : Blending(x0, x3, 1, data.n);
                evaluateAndUpdate(x_c);
                if (x_c.ofv < (outsite ? x_r.ofv : x3.ofv)) {
                    x3 = x_c;
                } else {
                    shrink = true;
                }
            }
            
            if (shrink) {
                x2 = Blending(x1, x2, 1, data.n);
                evaluateAndUpdate(x2);
                
                x3 = Blending(x1, x3, 1, data.n);
                evaluateAndUpdate(x3);
            }

            // Sort again
            sortSimplex(x1, x2, x3);

            x0 = Blending(x1, x2, 1, data.n);
            evaluateAndUpdate(x0);

            if (improved) {
                improved = 0;
                iter_count = 0;
            } else {
                iter_count++;
            }

            if (stop_execution.load()) return; 
        }

        x1 = improvedX1 ? xBest : x1Origem;
    }

    void SwapLS(TSol &s, const TProblemData &data, const int &strategy, std::vector<int> &RKorder)
    {                 
        std::shuffle(RKorder.begin(), RKorder.end(), rng);
        float rate = 1.0;
        
        TSol sBest = s;
        TSol sCurrent = s; // Usado no strategy 2
        bool improved = false;

        // Limites do loop ajustados para segurança
        int limitI = (int)((data.n - 1) * rate);
        int limitJ = (int)(data.n * rate);

        if (strategy == 1) { // First Improvement
            for(int i = 0; i < limitI; i++) {
                for(int j = i + 1; j < limitJ; j++) {  
                    std::swap(s.rk[RKorder[i]], s.rk[RKorder[j]]);
                    s.ofv = Decoder(s, data);

                    if (s.ofv < sBest.ofv){
                        sBest = s;
                        // return; // Se descomentar, vira First Impr real
                    } else {
                        s = sBest; // Reverte
                    }
                    if (stop_execution.load()) return; 
                }
            }
        }
        else if (strategy == 2) { // Best Improvement
            for(int i = 0; i < limitI; i++) {
                for(int j = i + 1; j < data.n; j++) {  
                    std::swap(s.rk[RKorder[i]], s.rk[RKorder[j]]);
                    s.ofv = Decoder(s, data);

                    if (s.ofv < sBest.ofv){
                        sBest = s;
                        improved = true;
                    }
                    s = sCurrent; // Sempre reverte para testar o próximo swap a partir da original
                    if (stop_execution.load()) return; 
                }
            }
            if (improved) s = sBest;
        }
    }

    void InvertLS(TSol &s, const TProblemData &data, const int &strategy, std::vector<int> &RKorder)
    {         
        std::shuffle(RKorder.begin(), RKorder.end(), rng);
        float rate = 1.0;
        int limit = (int)(data.n * rate);
        
        TSol sBest = s;
        TSol sCurrent = s;
        bool improved = false;

        for(int i = 0; i < limit; i++) {
            // invert logic
            if (s.rk[RKorder[i]] > 0.00001) s.rk[RKorder[i]] = 1.0 - s.rk[RKorder[i]];
            else s.rk[RKorder[i]] = 0.99999;

            s.ofv = Decoder(s, data);

            if (s.ofv < sBest.ofv){
                sBest = s;
                if(strategy == 1) { 
                    // s = sBest já é verdade aqui
                } else {
                    improved = true;
                }
            } else {
                if (strategy == 1) s = sBest; // Reverte se first impr falhou
            }

            if (strategy == 2) s = sCurrent; // Reverte sempre no best impr
            if (stop_execution.load()) return; 
        }

        if (strategy == 2 && improved) s = sBest;
    }

    void FareyLS(TSol &s, const TProblemData &data, const int &strategy, std::vector<int> &RKorder)
    {      
        std::shuffle(RKorder.begin(), RKorder.end(), rng);
        
        static const std::vector<double> F = {
            0.00, 0.142857, 0.166667, 0.20, 0.25, 0.285714, 0.333333, 0.40, 
            0.428571, 0.50, 0.571429, 0.60, 0.666667, 0.714286, 0.75, 0.80, 
            0.833333, 0.857143, 1.0
        };
                             
        float rate = 1.0;
        int limit = (int)(data.n * rate);
        
        TSol sBest = s;
        TSol sCurrent = s;
        bool improved = false;

        for(int i = 0; i < limit; i++) {
            for (size_t j = 0; j < F.size() - 1; j++){
                s.rk[RKorder[i]] = randomico(F[j], F[j+1]);
                s.ofv = Decoder(s, data);

                if (s.ofv < sBest.ofv){
                    sBest = s;
                    if(strategy == 2) improved = true;
                } else {
                    if(strategy == 1) s = sBest;
                }
                
                if (strategy == 2) s = sCurrent;
                if (stop_execution.load()) return; 
            }
        }
        if (strategy == 2 && improved) s = sBest;
    }

    void RVND(TSol &s, const TProblemData &data, const int &strategy, std::vector<int> &RKorder)
    {
        int numLS = 4;
        std::vector<int> NSL;
        for (int i = 1; i <= numLS; i++) NSL.push_back(i);

        while (!NSL.empty())
        {
            if (stop_execution.load()) return;       

            double foCurrent = s.ofv;
            
            int pos = rand() % NSL.size(); // rand() aqui é C-style, mas mantive do original
            int k = NSL[pos];

            switch (k)
            {
                case 1: SwapLS(s, data, strategy, RKorder); break;
                case 2: InvertLS(s, data, strategy, RKorder); break;
                case 3: NelderMeadSearch(s, data); break;
                case 4: FareyLS(s, data, strategy, RKorder); break;
            }

            if (s.ofv < foCurrent){
                NSL.clear();
                for (int i = 1; i <= numLS; i++) NSL.push_back(i);
            } else {
                NSL.erase(NSL.begin() + pos);
            }
        } 
    }

    void readParameters(const char* method, int control, 
                        std::vector<std::vector<double>> &parameters, int numPar)
    {
        #pragma omp critical
        {  
            char paramFile[256];
            if (control == 0) strncpy(paramFile, "config/param-offline.txt", 255);
            else strncpy(paramFile, "config/param-online.txt", 255);

            FILE *file = fopen(paramFile, "r");
            if (file == NULL) {
                printf("Error in open file %s.\n", paramFile);
                exit(1); // Mudei getchar() para exit para não travar automação
            }

            char line[1024]; 
            while (fgets(line, sizeof(line), file) != NULL) {
                line[strcspn(line, "\n")] = '\0'; 

                if (strcmp(line, method) == 0) {
                    for (int i = 0; i < numPar; i++) {
                        if (fgets(line, sizeof(line), file) != NULL) {
                            char *token = strtok(line, "{},= "); 
                            while (token != NULL) {
                                double aux = 0;
                                if (sscanf(token, "%lf", &aux) == 1) {
                                    parameters[i].push_back(aux);
                                }
                                token = strtok(NULL, "{},= "); 
                            }
                        }
                    }
                }
            }
            fclose(file);
        }
    }

    void readParametersYaml(const char* method, int control, 
                    std::vector<std::vector<double>> &parameters, int numPar)
    {
        #pragma omp critical
        {
            // 1. Definição do arquivo (agora com extensão .yaml)
            std::string paramFile;
            if (control == 0) paramFile = "config/yaml/param-offline.yaml";
            else paramFile = "config/yaml/param-online.yaml";

            try {
                // 2. Carrega o arquivo usando yaml-cpp
                YAML::Node config = YAML::LoadFile(paramFile);

                // 3. Verifica se o método existe no YAML
                if (config[method]) {
                    const YAML::Node& methodNode = config[method];

                    // Verifica se é uma sequência (lista de listas)
                    if (methodNode.IsSequence()) {
                        // Itera até o limite de numPar ou o tamanho disponível no YAML
                        for (int i = 0; i < numPar && i < methodNode.size(); i++) {
                            
                            // Verifica se o item interno também é uma sequência (lista de doubles)
                            if (methodNode[i].IsSequence()) {
                                for (const auto& val : methodNode[i]) {
                                    parameters[i].push_back(val.as<double>());
                                }
                            }
                        }
                    } else {
                        std::cerr << "Formato inválido para o método: " << method << " (esperado uma lista de listas)." << std::endl;
                    }
                } else {
                    std::cerr << "Método '" << method << "' não encontrado em " << paramFile << std::endl;
                }

            } catch (const YAML::BadFile& e) {
                std::cerr << "Erro ao abrir o arquivo YAML: " << paramFile << std::endl;
                exit(1); 
            } catch (const YAML::ParserException& e) {
                std::cerr << "Erro de sintaxe no YAML: " << e.what() << std::endl;
                exit(1);
            } catch (const std::exception& e) {
                std::cerr << "Erro desconhecido: " << e.what() << std::endl;
                exit(1);
            }
        }
    }

} // namespace rkolib::core