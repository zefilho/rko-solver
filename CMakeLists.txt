cmake_minimum_required(VERSION 3.16)

# Nome do projeto e versão
project(RKOLib VERSION 1.0 LANGUAGES CXX)

# ------------------------------------------------------------------------------
# 1. Configurações Globais (C++20 e Otimizações)
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Organização de saída de binários
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)

# Encontrar OpenMP
find_package(OpenMP REQUIRED)

# Variável para biblioteca de link dinâmico (necessária para dlopen/dlsym no Linux)
set(DL_LIB ${CMAKE_DL_LIBS})

# ------------------------------------------------------------------------------
# 2. Gerenciamento de Dependências (Hermetic Build)
# ------------------------------------------------------------------------------
include(FetchContent)

# No seu CMakeLists.txt, substitua a parte do yaml-cpp por esta:
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        yaml-cpp-0.7.0  # Usar uma TAG fixa é crucial para estabilidade
)

# Forçar a desativação de funcionalidades pesadas do yaml-cpp
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(yaml-cpp)

# CLI11
FetchContent_Declare(
    cli11
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
    GIT_TAG        v2.3.2
)
FetchContent_MakeAvailable(cli11)

# ------------------------------------------------------------------------------
# 3. Definição da Biblioteca Core (Motor do Solver)
# ------------------------------------------------------------------------------
# CORREÇÃO: Listamos os arquivos do CORE explicitamente para evitar Undefined Reference.
# Certifique-se de que qlearning.cpp está nesta lista.
set(CORE_SOURCES
    "src/core/solver.cpp"
    "src/core/method.cpp"
    "src/core/qlearning.cpp"  # Essencial para as referências de ChooseAction/CreateStates
    "src/utils/io.cpp"
)

# Coleta todas as Meta-heurísticas em src/mh/
file(GLOB_RECURSE MH_SOURCES "src/mh/*.cpp")

# Cria a biblioteca estática combinando o Motor, as MHs e a lógica de RL
add_library(rkolib STATIC ${CORE_SOURCES} ${MH_SOURCES})

# Configura diretórios de include para que headers como common.hpp sejam achados
target_include_directories(rkolib PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Linka dependências no Core. Note que DL_LIB é linkada aqui para os plugins funcionarem.
target_link_libraries(rkolib PUBLIC 
    OpenMP::OpenMP_CXX
    yaml-cpp
    CLI11::CLI11
    ${DL_LIB}
)

# Flags de compilação: -fPIC é obrigatório para que o código estático funcione com plugins .so
target_compile_options(rkolib PRIVATE
    -Wall -Wextra -fPIC
    $<$<CONFIG:Release>:-O3 -march=native>
)

# ------------------------------------------------------------------------------
# 4. Executável Principal (Host)
# ------------------------------------------------------------------------------
add_executable(rkosolver src/main.cpp)
target_link_libraries(rkosolver PRIVATE rkolib)

# ------------------------------------------------------------------------------
# 5. O PLUGIN DO PROBLEMA (Módulo dinâmico)
# ------------------------------------------------------------------------------
# add_library MODULE impede que o linker tente carregar isso no início da execução
add_library(kpproblem MODULE src/core/kpproblem.cpp)

target_include_directories(kpproblem PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Remove o prefixo 'lib' no Linux (gera kpproblem.so)
set_target_properties(kpproblem PROPERTIES PREFIX "")