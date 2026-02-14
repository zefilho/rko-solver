/**
 * RKO Library - Main Entry Point
 * Clean implementation using Singleton pattern (no global variables)
 */

#include "rkolib/core/solver.hpp"
#include <iostream>
#include <exception>

// ============================================================================
// MAIN ENTRY POINT
// ============================================================================

int main(int argc, char* argv[]) {
    try {
        // Create solver instance
        rkolib::RkoSolver solver;
        
        // Configure solver (parses CLI arguments)
        solver.parseArguments(argc, argv);
        
        // Load configuration from file
        solver.loadConfiguration();
        
        // Execute optimization process
        solver.run();
        
        return EXIT_SUCCESS;
        
    } catch (const CLI::ParseError& e) {
        // CLI11 já imprimiu a mensagem de erro ou help
        // Retorna o código de saída apropriado
        return (e.get_exit_code() == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
        
    } catch (const std::exception& e) {
        std::cerr << "\n❌ ERROR: " << e.what() << "\n\n";
        return EXIT_FAILURE;
        
    } catch (...) {
        std::cerr << "\n❌ UNKNOWN ERROR occurred\n\n";
        return EXIT_FAILURE;
    }
}