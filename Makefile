.PHONY: all build clean plugin

# ------------------------------------------------------------------------------
# Variáveis Globais
# ------------------------------------------------------------------------------
BUILD_DIR  := build
PLUGIN_DIR := $(BUILD_DIR)/plugins
PROB_DIR    := problems

# Variáveis do Compilador para os Plugins (Fast Build)
CXX        := g++
CXXFLAGS   := -std=c++20 -O3 -Wall -Wextra -fPIC -shared
INCLUDES   := -I./include

all: build

build:
	@mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. && cmake --build .

run: 
	cd $(BUILD_DIR) && cmake .. && cmake --build .

clean:
	rm -rf $(BUILD_DIR)

# ------------------------------------------------------------------------------
# Regra de Plugins (Compilação Ágil e Isolada)
# ------------------------------------------------------------------------------
# Uso: make plugin PROB=tspproblem
plugin:
	@if [ -z "$(PROB)" ]; then \
		echo "=========================================================="; \
		echo "[Erro] Nome do arquivo de origem nao especificado."; \
		echo "Uso:   make plugin PROB=<arquivo_cpp_sem_extensao> [OUT=<nome_saida>]"; \
		echo "Ex:    make plugin PROB=tspproblem OUT=tspproblem"; \
		echo "=========================================================="; \
		exit 1; \
	fi
	@if [ ! -f "./$(PROB_DIR)/$(PROB).cpp" ]; then \
		echo "[Erro] Arquivo ./$(PROB_DIR)/$(PROB).cpp nao encontrado!"; \
		exit 1; \
	fi
	$(eval OUT_NAME := $(if $(OUT),$(OUT),$(PROB)))
	@mkdir -p $(PLUGIN_DIR)
	@echo "[Plugin] Compilando ./$(PROB_DIR)/$(PROB).cpp -> $(OUT_NAME).so"
	$(CXX) $(CXXFLAGS) $(INCLUDES) ./$(PROB_DIR)/$(PROB).cpp -o ./$(PLUGIN_DIR)/$(OUT_NAME).so
	@echo "[Sucesso] Plugin disponivel em: ./$(PLUGIN_DIR)/$(OUT_NAME).so"


#g++ -O3 -shared -fPIC -std=c++20 ./problems/tspproblem.cpp -o ./build/plugins/tspproblem.so