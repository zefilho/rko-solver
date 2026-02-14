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
        // Usa a variável global 'SOLVER_RNG' declarada como extern no .hpp
        return std::uniform_real_distribution<double>(min, max)(SOLVER_RNG);
    }

    int irandomico(int min, int max)
    {
        return (int)randomico(0, max - min + 1) + min;
    }

    double get_time_in_seconds() {
        #if defined(_WIN32) || defined(_WIN64)
            LARGE_INTEGER frequency; 
            LARGE_INTEGER timeCur;
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

    void CreatePoolSolutions(const IProblem &problem, const int sizePool)
    {
        // Nota: Acessa a variável global 'SOLVER_POOL'
        #pragma omp critical
        {
            for (int i = 0; i < sizePool; i++){
                CreateInitialSolutions(SOLVER_POOL[i], problem.getDimension());
                SOLVER_POOL[i].ofv = problem.evaluate(SOLVER_POOL[i]); // Chama Decoder externo
                SOLVER_POOL[i].best_time = get_time_in_seconds();
            }

            // sort SOLVER_POOL in increasing order of fitness
            std::sort(SOLVER_POOL.begin(), SOLVER_POOL.begin() + sizePool, sortByFitness);

            // verify if similar solutions exist in the SOLVER_POOL
            int clone = 0;
            // Cuidado: loop reverso original mantido, mas verifique se sizePool <= SOLVER_POOL.size()
            for (int i = sizePool - 1; i > 0; i--){
                if (std::abs(SOLVER_POOL[i].ofv - SOLVER_POOL[i-1].ofv) < 1e-9){ // Comparação double segura
                    for (int j = 0; j < 0.2 * problem.getDimension(); j++){
                        int pos = irandomico(0, problem.getDimension() - 1);
                        SOLVER_POOL[i].rk[pos] = randomico(0, 1);
                    }
                    SOLVER_POOL[i].ofv = problem.evaluate(SOLVER_POOL[i]);
                    clone = 1;
                }
            }

            // sort SOLVER_POOL again if clones were mutated
            if (clone)
            {
                std::sort(SOLVER_POOL.begin(), SOLVER_POOL.begin() + sizePool, sortByFitness);
            }
        }
    }

    void UpdatePoolSolutions(TSol s, const char* mh, const int debug)
    {
        #pragma omp critical
        {   
            // Checks if it already exists in the SOLVER_POOL
            bool exists = std::ranges::any_of(SOLVER_POOL, [&](const auto& entry) {
                return std::abs(entry.ofv - s.ofv) < 1e-9;
            });

            // print that a new best solution was found
            if (!SOLVER_POOL.empty() && s.ofv < SOLVER_POOL[0].ofv && debug) 
            {
                int thread_id = omp_get_thread_num();   
                std::cout << std::format("\nBest solution: {:.10f} (Thread: {} - MH: {})", s.ofv, thread_id, mh);
            } 

            // Goes from back to front
            if (!exists)
            {
                s.best_time = get_time_in_seconds();
                s.nameMH = mh; // std::string assignment

                // insert the new solution preserving order (Insertion Sort logic)
                int i;
                for (i = static_cast<int>(SOLVER_POOL.size()) - 1; i > 0 && SOLVER_POOL[i - 1].ofv > s.ofv; i--) {
                    SOLVER_POOL[i] = SOLVER_POOL[i - 1]; // Push to the right
                }
                SOLVER_POOL[i] = s; 
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

    void NelderMeadSearch(TSol &x1, const IProblem &problem)
    {
        int poolSize = (int)SOLVER_POOL.size();
        if (poolSize < 2) return; // Segurança

        TSol x1Origem = x1;
        TSol xBest = x1;

        bool improved = 0;
        bool improvedX1 = 0; //Rastreia se x1 foi melhorado
        
        // Helper: Avalia uma solução candidata e atualiza o xBest se necessário
        auto evaluateAndUpdate = [&](TSol& candidate) {
            candidate.ofv = problem.evaluate(candidate);
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

        TSol x2 = SOLVER_POOL[k1];
        TSol x3 = SOLVER_POOL[k2];

        // Sort x1, x2, x3
        sortSimplex(x1, x2, x3);
        
        // Centroid
        TSol x0 = Blending(x1, x2, 1, problem.getDimension());
        evaluateAndUpdate(x0);

        int iter_count = 1; 
        int maxIter = std::max(10, static_cast<int>(problem.getDimension() * std::exp(-2)));

        while (iter_count <= maxIter) 
        {
            bool shrink = false;

            // Reflection
            TSol x_r = Blending(x0, x3, -1, problem.getDimension());
            evaluateAndUpdate(x_r);

            if (x_r.ofv < x1.ofv) 
            {
                // Expansion
                TSol x_e = Blending(x_r, x0, -1, problem.getDimension());
                evaluateAndUpdate(x_e);
                x3 = (x_e.ofv < x_r.ofv) ? x_e : x_r;
            } 
            else if (x_r.ofv < x2.ofv) {
                x3 = x_r;
            } 
            else {
                bool outsite = x_r.ofv < x3.ofv;
                TSol x_c = outsite ? Blending(x_r, x0, 1, problem.getDimension()) : Blending(x0, x3, 1, problem.getDimension());
                evaluateAndUpdate(x_c);
                if (x_c.ofv < (outsite ? x_r.ofv : x3.ofv)) {
                    x3 = x_c;
                } else {
                    shrink = true;
                }
            }
            
            if (shrink) {
                x2 = Blending(x1, x2, 1, problem.getDimension());
                evaluateAndUpdate(x2);
                
                x3 = Blending(x1, x3, 1, problem.getDimension());
                evaluateAndUpdate(x3);
            }

            // Sort again
            sortSimplex(x1, x2, x3);

            x0 = Blending(x1, x2, 1, problem.getDimension());
            evaluateAndUpdate(x0);

            if (improved) {
                improved = 0;
                iter_count = 0;
            } else {
                iter_count++;
            }

            if (SOLVER_SHOULD_STOP) return; 
        }

        x1 = improvedX1 ? xBest : x1Origem;
    }

    void runFirstImprovement(rkolib::core::TSol &s, rkolib::core::TSol &sBest, 
                             const rkolib::core::IProblem &problem, 
                             const std::vector<int> &RKorder, int limitI, int limitJ) {
        for (int i = 0; i < limitI; ++i) {
            for (int j = i + 1; j < limitJ; ++j) {
                std::swap(s.rk[RKorder[i]], s.rk[RKorder[j]]);
                s.ofv = problem.evaluate(s);

                if (s.ofv < sBest.ofv) {
                    sBest = s;
                } else {
                    s = sBest; // Reverte
                }
                
                if (SOLVER_SHOULD_STOP) return;
            }
        }
    }

    void runBestImprovement(rkolib::core::TSol &s, rkolib::core::TSol &sBest, 
                            const rkolib::core::IProblem &problem, 
                            const std::vector<int> &RKorder, int limitI, int n) {
        bool improved = false;
        rkolib::core::TSol sCurrent = s;

        for (int i = 0; i < limitI; ++i) {
            for (int j = i + 1; j < n; ++j) {
                std::swap(s.rk[RKorder[i]], s.rk[RKorder[j]]);
                s.ofv = problem.evaluate(s);

                if (s.ofv < sBest.ofv) {
                    sBest = s;
                    improved = true;
                }
                s = sCurrent; // Always revert to test the next neighbot
                
                if (SOLVER_SHOULD_STOP) return;
            }
        }
        if (improved) s = sBest;
    }

    void SwapLS(TSol &s, const IProblem &problem, const int &strategy, std::vector<int> &RKorder)
    {                 
        std::shuffle(RKorder.begin(), RKorder.end(), SOLVER_RNG);
        rkolib::core::TSol sBest = s;
        const float rate = 1.0f;
        const int n = static_cast<int>(problem.getDimension());
        const int limitI = static_cast<int>((n - 1) * rate);
        const int limitJ = static_cast<int>(n * rate);

        if (strategy == 1) {
            runFirstImprovement(s, sBest, problem, RKorder, limitI, limitJ);
        } 
        else if (strategy == 2) {
            runBestImprovement(s, sBest, problem, RKorder, limitI, n);
        }
    }

    void InvertLS(TSol &s, const IProblem &problem, const int &strategy, std::vector<int> &RKorder)
    {         
        std::shuffle(RKorder.begin(), RKorder.end(), SOLVER_RNG);
        float rate = 1.0;
        int limit = (int)(problem.getDimension() * rate);
        
        TSol sBest = s;
        TSol sCurrent = s;
        bool improved = false;

        for(int i = 0; i < limit; i++) {
            // invert logic
            if (s.rk[RKorder[i]] > 0.00001) s.rk[RKorder[i]] = 1.0 - s.rk[RKorder[i]];
            else s.rk[RKorder[i]] = 0.99999;

            s.ofv = problem.evaluate(s);

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
            if (SOLVER_SHOULD_STOP) return; 
        }

        if (strategy == 2 && improved) s = sBest;
    }

    void FareyLS(TSol &s, const IProblem &problem, const int &strategy, std::vector<int> &RKorder)
    {      
        std::shuffle(RKorder.begin(), RKorder.end(), SOLVER_RNG);
        
        static const std::vector<double> F = {
            0.00, 0.142857, 0.166667, 0.20, 0.25, 0.285714, 0.333333, 0.40, 
            0.428571, 0.50, 0.571429, 0.60, 0.666667, 0.714286, 0.75, 0.80, 
            0.833333, 0.857143, 1.0
        };
                             
        float rate = 1.0;
        int limit = static_cast<int>(problem.getDimension() * rate);
        
        TSol sBest = s;
        TSol sCurrent = s;
        bool improved = false;

        for(int i = 0; i < limit; i++) {
            for (size_t j = 0; j < F.size() - 1; j++){
                s.rk[RKorder[i]] = randomico(F[j], F[j+1]);
                s.ofv = problem.evaluate(s);

                if (s.ofv < sBest.ofv){
                    sBest = s;
                    if(strategy == 2) improved = true;
                } else {
                    if(strategy == 1) s = sBest;
                }
                
                if (strategy == 2) s = sCurrent;
                if (SOLVER_SHOULD_STOP) return; 
            }
        }
        if (strategy == 2 && improved) s = sBest;
    }

    void RVND(TSol &s, const IProblem &problem, const int &strategy, std::vector<int> &RKorder)
    {
        // Define the list of neighborhood structures
        const int numLS = 4;
        enum class LS { SWAP = 1, INVERT, NELDERMEAD, FAREY };

        std::vector<int> NSL;
        std::iota(NSL.begin(), NSL.end(), 1); // Push with values 1 to numLS

        while (!NSL.empty())
        {
            if (SOLVER_SHOULD_STOP) return;       

            double foCurrent = s.ofv;
            
            std::uniform_int_distribution<size_t> dist(0, numLS - 1);
            size_t pos = dist(SOLVER_RNG);
            auto k = (LS) NSL[pos];

            switch (k)
            {
                case LS::SWAP: SwapLS(s, problem, strategy, RKorder); break;
                case LS::INVERT: InvertLS(s, problem, strategy, RKorder); break;
                case LS::NELDERMEAD: NelderMeadSearch(s, problem); break;
                default: FareyLS(s, problem, strategy, RKorder); break;
            }

            if (s.ofv < foCurrent){
                NSL.resize(numLS);
                std::iota(NSL.begin(), NSL.end(), 1); //Push com valores 1 a numLS
            } else {
                NSL.erase(NSL.begin() + pos);
            }
        } 
    }

    void readParameters(const std::string& method, int control,
                    std::vector<std::vector<double>>& parameters, int numPar)
    {
        #pragma omp critical
        {
            std::string paramFile = (control == 0) ? "config/param-offline.txt"
                                                : "config/param-online.txt";

            std::ifstream file(paramFile);
            if (!file.is_open()) {
                std::cerr << "Error in open file " << paramFile << ".\n";
                std::exit(1);
            }

            std::string line;
            while (std::getline(file, line)) {
                if (line == method) {
                    for (int i = 0; i < numPar; i++) {
                        if (std::getline(file, line)) {
                            // Remove caracteres: { } , = e espaços
                            std::string cleaned;
                            for (char c : line) {
                                if (c != '{' && c != '}' && c != ',' && c != '=' && c != ' ') {
                                    cleaned += c;
                                } else if (!cleaned.empty() && cleaned.back() != ' ') {
                                    cleaned += ' '; // Separa tokens com espaço
                                }
                            }

                            std::istringstream iss(cleaned);
                            std::string token;
                            while (iss >> token) {
                                double aux = 0;
                                try {
                                    size_t pos;
                                    aux = std::stod(token, &pos);
                                    if (pos == token.size()) { // Conversão completa
                                        parameters[i].push_back(aux);
                                    }
                                } catch (const std::invalid_argument&) {
                                    // Token não é um número válido, ignora
                                } catch (const std::out_of_range&) {
                                    // Valor fora do intervalo de double, ignora
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // void readParameters(const char*  method, int control, 
    //                 std::vector<std::vector<double>> &parameters, int numPar)
    // {
    //     #pragma omp critical
    //     {  
    //         char paramFile[256];
    //         if (control == 0){
    //             strncpy(paramFile,"config/param-offline.txt",255);
    //         }
    //         else{
    //             strncpy(paramFile,"config/param-online.txt",255);
    //         }

    //         FILE *file = fopen(paramFile, "r");
    //         if (file == NULL) {
    //             printf("Error in open file %s.\n", paramFile);
    //             getchar();
    //         }

    //         char line[100];     // Buffer line

    //         // Reading the file line by line
    //         while (fgets(line, sizeof(line), file) != NULL) {
    //             line[strcspn(line, "\n")] = '\0';  // remove newline

    //             if (strcmp(line, method) == 0) {
    //                 // Read the parameter values
    //                 for (int i = 0; i < numPar; i++) {
    //                     double aux = 0;
    //                     if (fgets(line, sizeof(line), file) != NULL) {
    //                         char *token = strtok(line, "{},= "); // separete line by delimiters
    //                         while (token != NULL) {
    //                             if (sscanf(token, "%lf", &aux) == 1) {
    //                                 parameters[i].push_back(aux);
    //                             }
    //                             token = strtok(NULL, "{},= "); // next value
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //         fclose(file);
    //     }
    // }

    void LoadYamlLogic(const std::string& paramFile, const char* method, int numPar, std::vector<std::vector<double>> &parameters) 
    {
        try {
            
            YAML::Node config = YAML::LoadFile(paramFile);

            // Guard Clause 1: Método não existe
            if (!config[method]) {
                std::cerr << "Método '" << method << "' não encontrado em " << paramFile << std::endl;
                return; // AGORA PERMITIDO (sai da função auxiliar, não do bloco critical diretamente)
            }

            const YAML::Node& methodNode = config[method];

            // Guard Clause 2: Formato inválido
            if (!methodNode.IsSequence()) {
                std::cerr << "Formato inválido para o método: " << method << " (esperado uma lista de listas)." << std::endl;
                return;
            }

            // Lógica de preenchimento
            int limit = std::min(numPar, static_cast<int>(methodNode.size()));
            
            for (int i = 0; i < limit; i++) {
                if (!methodNode[i].IsSequence()) continue;

                for (const auto& val : methodNode[i]) {
                    parameters[i].push_back(val.as<double>());
                }
            }

        } catch (const YAML::BadFile& e) {
            std::cerr << "Erro ao abrir o arquivo YAML: " << paramFile << std::endl;
            exit(1); // Exit é permitido pois mata o processo inteiro
        } catch (const YAML::ParserException& e) {
            std::cerr << "Erro de sintaxe no YAML: " << e.what() << std::endl;
            exit(1);
        } catch (const std::exception& e) {
            std::cerr << "Erro desconhecido: " << e.what() << std::endl;
            exit(1);
        }
    }

    void readParametersYaml(const char* method, int control, std::vector<std::vector<double>> &parameters, int numPar)
    {
        // Define o nome do arquivo fora da área crítica (operação leve)
        std::string paramFile = (control == 0) ? "config/yaml/param-offline.yaml" : "config/yaml/param-online.yaml";

        #pragma omp critical
        {
            // Chama a lógica. Se ela der return, ela volta para esta linha,
            // atinge o fechamento da chave '}' e libera o Lock corretamente.
            LoadYamlLogic(paramFile, method, numPar, parameters);
        }
    }

    // void readParametersYaml(const char* method, int control, 
    //                 std::vector<std::vector<double>> &parameters, int numPar)
    // {
    //     #pragma omp critical
    //     {
    //         // 1. Define the file (now with .yaml extension)
    //         std::string paramFile;
    //         if (control == 0) paramFile = "config/yaml/param-offline.yaml";
    //         else paramFile = "config/yaml/param-online.yaml";

    //         try {
                
    //             // 2. Loads the file using yaml-cpp
    //             YAML::Node config = YAML::LoadFile(paramFile);

    //             // 3. Verified if the method exists in the YAML
    //             if (!config[method]) {
    //                 std::cerr << "Método '" << method << "' não encontrado em " << paramFile << std::endl;
    //                 return;
    //             }

    //             const YAML::Node& methodNode = config[method];

    //             if (!methodNode.IsSequence()) {
    //                 std::cerr << "Formato inválido para o método: " << method << " (esperado uma lista de listas)." << std::endl;
    //                 return;
    //             }

    //             int limit = std::min(numPar, static_cast<int>(methodNode.size()));
    //             for (int i = 0; i < limit; i++) {
    //                 // Verified if the inner item is also a sequence (list of doubles)
    //                 if (!methodNode[i].IsSequence()) continue;

    //                 for (const auto& val : methodNode[i]) {
    //                     parameters[i].push_back(val.as<double>());
    //                 }
    //             }

    //         } catch (const YAML::BadFile& e) {
    //             std::cerr << "Erro ao abrir o arquivo YAML: " << paramFile << std::endl;
    //             exit(1); 
    //         } catch (const YAML::ParserException& e) {
    //             std::cerr << "Erro de sintaxe no YAML: " << e.what() << std::endl;
    //             exit(1);
    //         } catch (const std::exception& e) {
    //             std::cerr << "Erro desconhecido: " << e.what() << std::endl;
    //             exit(1);
    //         }
    //     }
    // }

} // namespace rkolib::core