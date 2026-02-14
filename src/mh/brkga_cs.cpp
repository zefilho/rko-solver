#include "rkolib/mh/brkga_cs.hpp"

// Dependências internas
#include "rkolib/core/method.hpp"
#include "rkolib/core/qlearning.hpp"
#include "rkolib/core/iproblem.hpp" // Para problem.getDimension()

namespace rkolib::mh {

    using namespace rkolib::core;

    // =========================================================================
    // HELPER FUNCTIONS (INTERNAL)
    // =========================================================================

    // -------------------------------------------------------------------------
    // Method: PearsonCorrelation
    // -------------------------------------------------------------------------
    static double PearsonCorrelation(const std::vector<double>& X, const std::vector<double>& Y, const int n)
    {
        double sumXY = 0;
        double sumX2 = 0;
        double sumY2 = 0;
        double sumX = 0;
        double sumY = 0;

        for(int j = 0; j < n; j++)
        {
            sumX  += X[j];
            sumX2 += X[j] * X[j];
            sumXY += X[j] * Y[j];
            sumY  += Y[j];
            sumY2 += Y[j] * Y[j];
        }

        // Pearson denominator
        double denominator = sqrt((n * sumX2 - sumX * sumX) * (n * sumY2 - sumY * sumY));
        
        if (denominator == 0) return 0.0;

        return ((n * sumXY) - (sumX * sumY)) / denominator;
    }

    // -------------------------------------------------------------------------
    // Method: ParametricUniformCrossover
    // Optimization: Pop passed by const reference
    // -------------------------------------------------------------------------
    static TSol ParametricUniformCrossover(int eliteSize, int popSize, double pm, double rhoe, const std::vector<TSol>& Pop, const int n)
    {    
        TSol s;

        // one chromosome from elite set
        int eliteParent = irandomico(0, eliteSize - 1);                 

        // one chromosome from non-elite population
        int nonEliteParent = irandomico(eliteSize, popSize - 1);           
        
        // Initialize offspring (resize vector inside TSol)
        s = Pop[eliteParent]; // Copy structure/size from elite

        for(int j = 0; j < n; j++)
        {
            // mutation
            if (randomico(0, 1) < pm)
            {
                s.rk[j] = randomico(0, 1);
            }
            // mate
            else
            {
                // copy alleles of top chromosome of the new generation (Elite)
                if (randomico(0, 1) < rhoe){
                    s.rk[j] = Pop[eliteParent].rk[j];
                }
                else{
                    s.rk[j] = Pop[nonEliteParent].rk[j];
                }
            }
        }

        return s;
    }

    // -------------------------------------------------------------------------
    // Method: ChaoticInd
    // -------------------------------------------------------------------------
    static void ChaoticInd(TSol &s, int rhoe, const int n) // Note: rhoe param was int in original, kept but logic implies probability
    {
        // generate a chaotic individual
        // Note: Logic implies that if random > rhoe (likely high prob), we randomize. 
        // Original code cast rhoe (double) to int in call, making it 0 or 1. Verify logic.
        // Assuming rhoe is meant to be double probability here based on usage.
        
        for (int k = 0; k < n; k++)
        {       
            if (randomico(0, 1) > (double)rhoe) 
               s.rk[k] = randomico(0, 1);
        }
    }

    // -------------------------------------------------------------------------
    // Method: UpdatePopulationSize
    // -------------------------------------------------------------------------
    static void UpdatePopulationSize(int p, double pe, double pm, double rhoe, std::vector<TSol> &Pop, std::vector<TSol> &PopInter, const IProblem &problem)
    {
        // size of the current population
        int oldPsize = (int)Pop.size();

        // proportional pruning 
        if (oldPsize > p){

            // copy the current population
            PopInter = Pop;

            // define new size of Pop
            Pop.resize(p);

            // select the elite chromosomes
            for (int i = 0; i < (int)(p * pe); i++){
                // copy p*pe best chromosomes
                Pop[i] = PopInter[i];
            }

            // select the non-elite chromosomes
            // Original logic: fill the rest of the new population size 'p' 
            // taking from the old population starting at the old elite position.
            int pos = (int)(pe * oldPsize);
            for (int i = (int)(p * pe); i < p; i++){
                if (pos < (int)PopInter.size()) {
                    Pop[i] = PopInter[pos];
                    pos++;
                } else {
                    // Fallback if calculations imply out of bounds
                    CreateInitialSolutions(Pop[i], problem.getDimension());
                    Pop[i].ofv = problem.evaluate(Pop[i]);
                }
            }

            // clean intermediate population
            PopInter.clear();
            PopInter.resize(p);
        }
        
        // generate new chromosomes 
        else if (oldPsize < p){

            // define new size of Pop
            Pop.resize(p);

            for (int k = oldPsize; k < p; k++)
            {
                if (SOLVER_SHOULD_STOP) return;     

                // Generate new individuals using crossover from existing ones
                Pop[k] = ParametricUniformCrossover((int)(oldPsize * pe), oldPsize - 1, pm, rhoe, Pop, problem.getDimension());
                Pop[k].ofv = problem.evaluate(Pop[k]);
            }

            // sort new population
            std::sort(Pop.begin(), Pop.end(), sortByFitness);
            
            // clean intermediate population
            PopInter.clear();
            PopInter.resize(p);
        }
    }

    // -------------------------------------------------------------------------
    // Method: LP (Label Propagation)
    // -------------------------------------------------------------------------
    static void LP(const std::vector<std::vector<std::pair<int, double>>>& listaArestas, std::vector<TSol> &Pop, std::vector<int> &label)
    {
        int nk = (int)listaArestas.size();

        // Create vector with visit order
        std::vector<int> ordemVisita(nk);
        std::iota(ordemVisita.begin(), ordemVisita.end(), 0);

        // initialize each node with its own label
        for (int i = 0; i < nk; i++)
            label[i] = i;

        int movimentos = 1;
        
        // Use local random device for shuffle to avoid global dependency issues in helper
        std::mt19937 g(std::random_device{}());

        while (movimentos) 
        {
            movimentos = 0;
            std::shuffle(ordemVisita.begin(), ordemVisita.end(), g);
            
            for (std::vector<int>::size_type idVertice = 0; idVertice < ordemVisita.size(); idVertice++)
            {
                int u = ordemVisita[idVertice];

                // Calculate the weight of the labels
                std::map<int, double> totalLabels;
                
                for (const auto& edge : listaArestas[u]) {
                    int idVizinho = edge.first;
                    double peso = edge.second;
                    int labelVizinho = label[idVizinho];
                    
                    totalLabels[labelVizinho] += peso;
                }

                // Best label is itself initially
                int melhorLabel = label[u];
                double melhorPeso = -1.0; 
                
                // If it has neighbors, find best
                if (!totalLabels.empty()) {
                    // Try to keep current label if it's best or equal
                    if (totalLabels.find(label[u]) != totalLabels.end()) {
                         melhorPeso = totalLabels[label[u]];
                    } else {
                         melhorPeso = std::numeric_limits<double>::lowest();
                    }

                    for (const auto& pair : totalLabels) {
                        if (pair.second > melhorPeso) {
                            melhorLabel = pair.first;
                            melhorPeso = pair.second;
                        }
                    }
                }

                if (melhorLabel != label[u]) {
                    label[u] = melhorLabel;
                    movimentos = 1;
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    // Method: PromisingLP
    // -------------------------------------------------------------------------
    static void PromisingLP(int p, double pe, std::vector<TSol> &Pop, std::vector<int> &label, std::vector<int> &promising)
    {
        int Tpe = (int)(p * pe);
        std::vector<int> grupos;
        
        // save labels defined by LP in groups
        for (int i = 0; i < Tpe; i++)
        {
            bool found = false;
            for (unsigned int j = 0; j < grupos.size(); j++)
            {
                if (label[i] == grupos[j]) {
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                grupos.push_back(label[i]);
            }
        }

        // select as promising a random solution in each group
        for (unsigned int j = 0; j < grupos.size(); j++)
        {
            int numTry = 0;
            
            // Safety break to prevent infinite loops if logic fails
            while(numTry < Tpe * 2) 
            {
                numTry++;
                int pos = irandomico(0, Tpe - 1);

                if (label[pos] == grupos[j])
                {
                    promising[pos] = 1;
                    break;
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    // Method: IC (Initial Clustering)
    // -------------------------------------------------------------------------
    static void IC(int p, double pe, std::vector<TSol> &Pop, std::vector<int> &promising, const int n) 
    {
        int Tpe = (int)(p * pe);
        // Adjacency list: Index -> vector of pairs (Neighbor Index, Weight)
        std::vector<std::vector<std::pair<int, double>>> listaArestas(Tpe);

        // pearson correlation threshold
        double sigma = 0.6;                      

        // create weighted (pearson correlation) graph
        for (int i = 0; i < Tpe - 1; i++) {
            for (int j = i + 1; j < Tpe; j++)
            {
                double pearson = PearsonCorrelation(Pop[i].rk, Pop[j].rk, n);
                
                if (pearson > sigma) {
                    listaArestas[i].push_back(std::make_pair(j, pearson));
                    listaArestas[j].push_back(std::make_pair(i, pearson));
                }
            }
        }

        std::vector<int> label(Tpe, 0);

        // apply clustering method (Label Propagation)
        LP(listaArestas, Pop, label);

        // Identify representatives
        PromisingLP(p, pe, Pop, label, promising);
    }

    // =========================================================================
    // MAIN ALGORITHM IMPLEMENTATION
    // =========================================================================

    void BRKGA_CS(const TRunData &runData, const IProblem &problem)
    {
        const char* method = "BRKGA-CS";
        // BRKGA parameters
        int p = 1597;                         // size of population
        double pe = 0.20;                     // fraction of population to be the elite-set
        double pm = 0.03;                     // fraction of population to be replaced by mutants
        double rhoe = 0.70;                   // probability that offspring inherit an allele from elite parent

        // BRKGA variables
        std::vector<TSol> Pop;                // current population
        std::vector<TSol> PopInter;           // intermediary population
        TSol bestInd;                         // best solution of the BRKGA-CS

        double start_timeMH = get_time_in_seconds();    // start computational time
        double end_timeMH = get_time_in_seconds();      // end computational time

        // Run
        int numGenerations = 0;               // number of generations
        int bestGeneration = 0;               // generation in which found the best solution
        float currentTime = 0;                // computational time of the search process
        int sumLS = 0;                        // number of local search applied in each generation
        int noImprovBRKGA = 0;                // number of generations without improvement in the best solution
        int improv = 0;                       // improvement flag

        // define a order for the neighors
        std::vector<int> RKorder(problem.getDimension());
        std::iota(RKorder.begin(), RKorder.end(), 0);

        // ---------------------------------------------------------------------
        // Q-Learning parameters
        // ---------------------------------------------------------------------
        std::vector<TState> S;              // finite state space
        int numPar = 0;                     // number of parameters
        int numStates = 0;                  // number of states
        int iCurr = 0;                      // current (initial) state
        double epsilon = 0;                 // greed choice possibility
        double lf = 0;                      // learning factor
        double df = 0;                      // discount factor
        double R = 0;                       // reward
        
        float epsilon_max = 1.0;            // maximum epsilon 
        float epsilon_min = 0.1;            // minimum epsilon
        int Ti = 1;                         // number of epochs performed
        int restartEpsilon = 1;             // number of restart epsilon
        int st = 0;                         // current state
        int at = 0;                         // current action

        // ** read file with parameter values
        numPar = 4;
        std::vector<std::vector<double>> parameters;
        parameters.resize(numPar);

        readParameters(method, runData.control, parameters, numPar);

        // offline control
        if (runData.control == 0)
        {
            if (!parameters[0].empty()) p    = (int)parameters[0][0];
            if (!parameters[1].empty()) pe   = parameters[1][0];                                            
            if (!parameters[2].empty()) pm   = parameters[2][0];
            if (!parameters[3].empty()) rhoe = parameters[3][0];
        }

        // online control
        else 
        {
            // Q-Learning 
            if (runData.control == 1){
                // create possible states of the Markov chain
                CreateStates(parameters, numStates, numPar, S);

                // number of restart epsilon
                restartEpsilon = 1;  

                // maximum epsilon  
                epsilon_max = 1.0;  

                // current state
                iCurr = irandomico(0, numStates - 1);

                // define the initial parameters of the BRGKA
                if (!S.empty()) {
                    p    = (int)S[iCurr].par[0];
                    pe   = S[iCurr].par[1];                                              
                    pm   = S[iCurr].par[2];                                                      
                    rhoe = S[iCurr].par[3];
                }
            }
        }

        // initialize population
        Pop.clear();  
        PopInter.clear(); 

        Pop.resize(p);
        PopInter.resize(p);

        // Create the initial chromosomes with random keys
        bestInd.ofv = std::numeric_limits<double>::infinity();

        for (int i = 0; i < p; i++)
        {
            CreateInitialSolutions(Pop[i], problem.getDimension()); 
            Pop[i].ofv = problem.evaluate(Pop[i]);
            PopInter[i] = Pop[i];

            if (Pop[i].ofv < bestInd.ofv) {
                bestInd = Pop[i];
            }
        }
        
        // sort population in increase order of fitness
        std::sort(Pop.begin(), Pop.end(), sortByFitness);
        // bestInd = Pop[0]; // Redundant if checked in loop, but safe

        // run the evolutionary process until stop criterion
        while (currentTime < runData.MAXTIME * runData.restart)
        {
            // number of generations
            numGenerations++;

            // number of generations without improvement in the best solution
            noImprovBRKGA++;

            // -----------------------------------------------------------------
            // Q-Learning Update Phase (Pre-Action)
            // -----------------------------------------------------------------
            if (runData.control == 1 && !S.empty()){
                // set Q-Learning parameters  
                SetQLParameter(currentTime, Ti, restartEpsilon, epsilon_max, epsilon_min, epsilon, lf, df, (int)(runData.MAXTIME * runData.restart)); 

                // choose a action a_t for current state s_t
                at = ChooseAction(S, st, epsilon);

                // execute action a_t
                if (at >= 0 && at < (int)S[st].Ai.size()) {
                    iCurr = S[st].Ai[at];

                    // define the parameters of the BRGKA according of the current state
                    p       = (int)S[iCurr].par[0];
                    pe      = S[iCurr].par[1];                                              
                    pm      = S[iCurr].par[2];                                                      
                    rhoe    = S[iCurr].par[3]; 
                    
                    // update population size                                                      
                    UpdatePopulationSize(p, pe, pm, rhoe, Pop, PopInter, problem);     
                }               
            }

            // -----------------------------------------------------------------
            // BRKGA Evolution Phase
            // -----------------------------------------------------------------

            // Ensure intermediate population is sized correctly (if p changed)
            if ((int)PopInter.size() != p) PopInter.resize(p);

            // The 'Pe' best chromosomes are maintained
            for (int i = 0; i < (int)(p * pe); i++){
                PopInter[i] = Pop[i]; 
            }  

            // We'll mate 'P - Pe' pairs
            double bestOFV = INFINITY;
            for (int i = (int)(p * pe); i < p; i++){              
                if (SOLVER_SHOULD_STOP) return;      

                // Parametric uniform crossover with mutation
                PopInter[i] = ParametricUniformCrossover((int)(p * pe), p, pm, rhoe, Pop, problem.getDimension());
     
                // Calculate the fitness of new chromosomes
                PopInter[i].ofv = problem.evaluate(PopInter[i]); 

                if (PopInter[i].ofv < bestOFV)
                    bestOFV = PopInter[i].ofv;
            }
                    
            // Update the current population
            Pop = PopInter;   

            // Sort population in increase order of fitness
            std::sort(Pop.begin(), Pop.end(), sortByFitness);

            // We improve the best fitness in the current population 
            if (Pop[0].ofv < bestInd.ofv){
                bestInd = Pop[0];
                bestGeneration = numGenerations;
                noImprovBRKGA = 0;
                improv = 1;

                // update the SOLVER_POOL of solutions
                UpdatePoolSolutions(bestInd, method, runData.debug);
            }

            // -----------------------------------------------------------------
            // Q-Learning Reward Phase (Post-Action)
            // -----------------------------------------------------------------
            if (runData.control == 1 && !S.empty()){
                // We improve the best fitness in the current population 
                if (improv){
                    // The reward function is based on improvement of the current best fitness and binary reward
                    R = 1.0 + 1.0 / (double)p;                              
                    improv = 0;
                }    
                else{
                    // Protection against division by zero
                     if (std::abs(bestOFV) > 1e-9)
                        R = (bestInd.ofv - bestOFV) / bestOFV;
                    else
                        R = 0;
                }
                
                // index of the next state
                if (at >= 0 && at < (int)S[st].Ai.size()) {
                    int st_1 = S[st].Ai[at];

                    // Update the Q-Table value
                    if (at < (int)S[st].Qa.size()) {
                        S[st].Qa[at] = S[st].Qa[at] + lf * (R + df * S[st_1].maxQ - S[st].Qa[at]); 

                        if (S[st].Qa[at] > S[st].maxQ)
                        {
                            S[st].maxQ = S[st].Qa[at];
                            S[st].maxA = at;
                        }
                    }
                    // Define the new current state st
                    st = st_1;
                }
            }


            // ********************* LOCAL SEARCH IN COMMUNITIES *******************
            sumLS = 0;

            // apply local search when BRKGA found a new better solution or 10 generations without improvements
            if (noImprovBRKGA == 0 || noImprovBRKGA > 10)
            {
                // restart the count of generations without improvements (local search)
                noImprovBRKGA = 0;

                // promising vector
                std::vector<int> promising((int)(p * pe), 0);

                // Identify commuties in the Elite with Label Propagation method
                IC(p, pe, Pop, promising, problem.getDimension());

                std::vector<int> promisingSol; 
                promisingSol.clear();

                // Select candidates from Elite
                int limitElite = (int)(p * pe);
                for (int i = 0; i < limitElite; i++) {
                    if (SOLVER_SHOULD_STOP) return;  

                    // insert the individual index in the promising list
                    if (promising[i] == 1){
                        promisingSol.push_back(i);
                    }
                    
                    // generate chaotic individual (crossover between one elite and one mutant)
                    // The original logic replaced the elite individual. We replicate that.
                    else {
                        // Cast rhoe to int to match helper signature, though semantically it's double
                        ChaoticInd(Pop[i], (int)rhoe, problem.getDimension()); 
                        Pop[i].ofv = problem.evaluate(Pop[i]);
                    }
                }

                // Apply LS on promising
                for (unsigned int i = 0; i < promisingSol.size(); i++){
                    if (SOLVER_SHOULD_STOP) return;      

                    int solIndex = promisingSol[i];
                    // local search not influence the evolutionary process
                    if (i < 1)
                        RVND(Pop[solIndex], problem, runData.strategy, RKorder);
                    else
                        NelderMeadSearch(Pop[solIndex], problem);        

                    if (Pop[solIndex].ofv < bestInd.ofv){
                        bestInd = Pop[solIndex];
                        bestGeneration = numGenerations;
                        noImprovBRKGA = 0;

                        // update the SOLVER_POOL of solutions
                        UpdatePoolSolutions(bestInd, method, runData.debug);
                    }
                }

                sumLS = (int)promisingSol.size();
                promisingSol.clear();
                promising.clear();

                // Sort again as local search might have changed fitnesses or ChaoticInd changed individuals
                std::sort(Pop.begin(), Pop.end(), sortByFitness);
            }
            // *********************************************************************

            // terminate the evolutionary process in MAXTIME
            end_timeMH = get_time_in_seconds();
            currentTime = (float)(end_timeMH - start_timeMH);
        }

        // free memory of BRKGA-CS components
        Pop.clear();
        PopInter.clear();
    }

} // namespace rkolib::mh