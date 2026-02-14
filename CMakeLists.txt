cmake_minimum_required(VERSION 3.16)

# Nome do projeto e versão
project(RKOLib VERSION 1.0 LANGUAGES CXX)

# ------------------------------------------------------------------------------
# 1. Configurações Globais (C++20 e Otimizações)
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Saída dos binários na pasta raiz (opcional, mas facilita achar o executável)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

# Encontrar dependências (OpenMP como pedido anteriormente)
find_package(OpenMP REQUIRED)

# # 1.2 yaml-cpp via FetchContent (Hermetic Build)
include(FetchContent)

FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        yaml-cpp-0.7.0  # Usar uma TAG fixa é crucial para estabilidade
)

# Opções para não compilar testes ou ferramentas extras do yaml-cpp (acelera o build)
set(YAML_CPP_BUILD_TESTS OFF CACHE INTERNAL "")
set(YAML_CPP_BUILD_TOOLS OFF CACHE INTERNAL "")
set(YAML_CPP_INSTALL OFF CACHE INTERNAL "") # Não precisamos instalar no sistema, apenas usar local

# Baixa e disponibiliza os targets (yaml-cpp)
FetchContent_MakeAvailable(yaml-cpp)

# 1.3 CLI11 via FetchContent
FetchContent_Declare(
    cli11
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
    GIT_TAG        v2.3.2
)
FetchContent_MakeAvailable(cli11)

# ------------------------------------------------------------------------------
# 2. Definição da Biblioteca (rkolib)
# ------------------------------------------------------------------------------
# Coleta automaticamente os .cpp dentro de src/core e src/mh
# Nota Senior: Em projetos gigantes, listamos arquivos manualmente. 
# Para libs médias, GLOB_RECURSE é muito prático.
file(GLOB_RECURSE LIB_SOURCES 
    "src/core/*.cpp"
    "src/utils/*.cpp"
    "src/mh/*.cpp"
)

# Cria a biblioteca estática (libRKOLib.a ou RKOLib.lib)
add_library(rkolib STATIC ${LIB_SOURCES})

# Configura os diretórios de include
# PUBLIC = Quem usar a lib (main.cpp) também enxerga essa pasta 'include'
target_include_directories(rkolib PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Linka OpenMP na biblioteca (se ela usar paralelismo interno)
target_link_libraries(rkolib PUBLIC 
    OpenMP::OpenMP_CXX
    yaml-cpp
    CLI11::CLI11
)

# Flags de compilação específicas da Lib
target_compile_options(rkolib PRIVATE
    -Wall -Wextra
    $<$<CONFIG:Release>:-O3 -march=native>
)

# ------------------------------------------------------------------------------
# 3. Definição do Executável (Main)
# ------------------------------------------------------------------------------
# Compila o src/main.cpp
add_executable(runApp src/main.cpp)

# Linka o executável com sua biblioteca recém-criada
target_link_libraries(runApp PRIVATE rkolib)

# ------------------------------------------------------------------------------
# 4. (Opcional) Testes
# ------------------------------------------------------------------------------
# Se a pasta 'tests' tiver um CMakeLists.txt próprio:
# enable_testing()
# add_subdirectory(tests)