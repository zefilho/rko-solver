#include "rkolib/core/solver.hpp"
#include <iostream>

// Definição das variáveis globais necessárias para compatibilidade legado
// (Se você refatorar as MHs no futuro para receberem contexto, isso some)
std::mt19937 rng;
std::vector<rkolib::core::TSol> pool;
std::atomic<bool> stop_execution;

int main(int argc, char *argv[]) {
    // 1. Instancia o Solver
    rkolib::RKOSolver rko;

    // 2. Inicializa (Lê CLI, Configs e Instância)
    int status = rko.init(argc, argv);
    
    // Se houve erro de CLI (ex: --help ou arquivo não encontrado), encerra
    if (status != 0) return (status > 0) ? 0 : 1;

    // 3. Executa a otimização
    rko.run();

    return 0;
}