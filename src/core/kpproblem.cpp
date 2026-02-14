#include "rkolib/core/iproblem.hpp"

using namespace rkolib::core;

class KnapsackProblem : public IProblem {
private:
    // Dados do problema
    int nItems;
    double cap;
    std::vector<double> w;
    std::vector<double> b;

    // 1. CONSTRUTOR PRIVADO
    // Ninguém pode dar "new KnapsackProblem()" exceto a própria classe.
    KnapsackProblem() : nItems(0), cap(0.0) {
        std::cout << "--- Singleton Inicializado ---" << std::endl;
    }

public:
    // 2. PROIBIR CÓPIA (Essencial para Singleton)
    KnapsackProblem(const KnapsackProblem&) = delete;
    void operator=(const KnapsackProblem&) = delete;

    // 3. MÉTODO DE ACESSO GLOBAL (Meyer's Singleton)
    // Thread-safe no C++11 em diante.
    static KnapsackProblem& getInstance() {
        static KnapsackProblem instance; // Criada apenas na primeira vez que passar aqui
        return instance;
    }

    // --- Implementação da Interface IProblem ---

    void load(const std::string& name) override {
        // Lógica de leitura (igual ao anterior)
        std::ifstream file(name);
        if (!file.is_open()) throw std::runtime_error("File not found");
        file >> nItems >> cap;
        w.resize(nItems); b.resize(nItems);
        for (int k = 0; k < nItems; k++) file >> b[k] >> w[k];
        
        std::cout << "Dados carregados no Singleton." << std::endl;
    }

    double evaluate(const TSol &s) const override {
        // Lógica de avaliação (igual ao anterior)
        double cost = 0;
        double totalW = 0;
        for (int i = 0; i < nItems; i++) {
            if (s.rk[i] > 0.5) {
                cost += b[i];
                totalW += w[i];
            }
        }
        double infeasible = std::max(0.0, totalW - cap);
        return (cost - (100000.0 * infeasible)) * -1.0;
    }

    int getDimension() const override {
        return nItems;
    }
};

// ---------------------------------------------------------
// A "FÁBRICA" ADAPTADA
// ---------------------------------------------------------
namespace rkolib::core {

    // Helper: Um "Deletor Falso". 
    // Quando o unique_ptr tentar destruir o Singleton, isso roda e não faz nada.
    struct NullDeleter {
        void operator()(IProblem*) const {
            // Não deletar! O Singleton vive na memória estática.
            std::cout << "--- Singleton preservado (NullDeleter) ---" << std::endl;
        }
    };

    // Agora retornamos um unique_ptr que aponta para o Singleton,
    // mas que sabe que NÃO deve deletá-lo.
    std::shared_ptr<IProblem> createProblem() {
        
        // Retorna um shared_ptr apontando para o Singleton.
        // O segundo argumento é o Deletor Personalizado (Lambda vazia).
        // Isso impede que o shared_ptr tente deletar a memória estática ao sair do escopo.
        return std::shared_ptr<IProblem>(
            &KnapsackProblem::getInstance(), 
            [](IProblem*) { /* Não faz nada! Singleton vive para sempre. */ }
        );
    }
}