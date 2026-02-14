#include "rkolib/utils/io.hpp"
#include "rkolib/core/data.hpp"
#include "rkolib/core/iproblem.hpp"

namespace rkolib::utils {

    void WriteSolutionScreen(const char *algorithms[], int numMH, rkolib::core::TSol s, 
                             float timeBest, float timeTotal, char instance[], 
                             const rkolib::core::IProblem &problem, std::vector<rkolib::core::TSol> pool)
    {
        printf("\n\n\nRKO: ");
        for (int i=0; i<numMH; i++)
            printf("%s | ", algorithms[i]);
        
        // s.nameMH agora é std::string (do passo anterior), usamos .c_str() para printf
        printf("\nBest MH: %s \nInstance: %s \nsol: ", s.nameMH.c_str(), instance);
        
        for (int i=0; i<problem.getDimension(); i++)
            printf("%.3lf ", s.rk[i]);

        printf("\nofv: %.5lf", s.ofv); 
        printf("\nTotal time: %.3f",timeTotal);
        printf("\nBest time: %.3f\n\n",timeBest);

        // print solution pool 
        printf("\nSolution Pool:\n");
        for (int i = 0; i< (int)pool.size(); i++)
            printf("%.5lf [%s]\n", pool[i].ofv, pool[i].nameMH.c_str());
    }

    void WriteSolution(const char *algorithms[], int numMH, rkolib::core::TSol s, 
                       float timeBest, float timeTotal, char instance[], 
                       const rkolib::core::IProblem &problem)
    {
        char name[256]="../Results/Solutions_RKO";
        strcat(name,".txt");

        // file to write the best solution found
        FILE *solFile;                       

        solFile = fopen(name,"a");

        if (!solFile)
        {
            printf("\n\nFile not found %s!!! (Please create 'Results' folder ../)", name);
            // getchar(); // Removido para não travar automação em servidores
            exit(1);
        }

        fprintf(solFile,"\n\nInstance: %s", instance);
        fprintf(solFile,"\nRKO: ");
        for (int i=0; i<numMH; i++)
            fprintf(solFile,"%s | ", algorithms[i]);

        fprintf(solFile,"\nSol: ");
        for (int i=0; i<problem.getDimension(); i++)
            fprintf(solFile,"%.3lf ", s.rk[i]);

        fprintf(solFile,"\nofv: %lf", s.ofv);
        fprintf(solFile,"\nBest time: %.3f",timeBest);
        fprintf(solFile,"\nTotal time:%.3f \n",timeTotal);

        fclose(solFile);
    }

    void WriteResults(const char *algorithms[], int numMH, double ofv, 
                      double ofvAverage, std::vector<double> ofvs, float timeBest, 
                      float timeTotal, char instance[])
    {
        char name[256]="../Results/Results_RKO";
        strcat(name,".csv");

        FILE *File;
        File = fopen(name,"a");

        if (!File)
        {
            printf("\n\nFile not found %s!!! (Please create 'Results' folder ../)", name);
            exit(1);
        }

        fprintf(File,"\n%s\t", instance);
        for (int i=0; i<numMH; i++)
            fprintf(File,"%s | ", algorithms[i]);

        fprintf(File,"\t%d", (int)ofvs.size());
        for (unsigned int i=0; i<ofvs.size(); i++){
            fprintf(File,"\t%lf", ofvs[i]);   
        }
        fprintf(File,"\t%lf", ofv);
        fprintf(File,"\t%lf", ofvAverage);
        fprintf(File,"\t%.3f", timeBest);
        fprintf(File,"\t%.3f", timeTotal);

        fclose(File);
    }

} // namespace rkolib::utils