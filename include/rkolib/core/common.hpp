#pragma once

// -------------------------------------------------------------------------
// 1. PLATFORM DETECTION & SYSTEM HEADERS
// -------------------------------------------------------------------------
#if defined(_WIN32) || defined(_WIN64)
    #define NOMINMAX // Evita conflitos com std::min/std::max no Windows
    #include <windows.h>
#else
    #include <sys/time.h>
    #include <ctime>
#endif

// -------------------------------------------------------------------------
// 2. STANDARD C++ LIBRARY (Commonly used across the project)
// -------------------------------------------------------------------------
// Containers
#include <vector>
#include <string>
#include <map>
#include <utility> // std::pair
#include <ranges> // C++20 ranges for convenience
#include <functional> // std::function

// Math & Algorithms
#include <cmath>
#include <algorithm>
#include <numeric> // std::iota, std::accumulate
#include <limits>  // std::numeric_limits

// IO & Strings
#include <format>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <cstring>
#include <cstdio> // printf (se necessário debug)
#include <cstdint> // Para tipos de tamanho fixo se necessário
#include <cstdlib>  // exit

// Concurrency & Randoms
#include <atomic>
#include <random>
#include <chrono>
#include <ctime>

#include <memory> // std::unique_ptr



// -------------------------------------------------------------------------
// 3. GLOBAL CONSTANTS & MACROS
// -------------------------------------------------------------------------
#ifndef INFINITY
    #define INFINITY std::numeric_limits<double>::infinity()
#endif
