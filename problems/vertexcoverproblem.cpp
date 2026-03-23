#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>

namespace rkolib::core {

struct TSol {
    std::vector<double> rk;
    double ofv = std::numeric_limits<double>::infinity();
    double best_time = 0.0;
    std::string nameMH;
    std::vector<double> objs;
    TSol() = default;
};

class IProblem {
public:
    virtual ~IProblem() = default;
    virtual void load(const std::string &filename) = 0;
    virtual void decode(TSol &sol) const = 0;
    virtual int getDimension() const = 0;
    virtual int getNumObjectives() const = 0;
};
} // namespace rkolib::core

class VertexCoverProblem : public rkolib::core::IProblem {
private:
    int nVertices;
    int nEdges;
    std::vector<std::pair<int, int>> edges;

public:
    VertexCoverProblem() : nVertices(0), nEdges(0) {}
    ~VertexCoverProblem() override = default;

    void load(const std::string &name) override {
        std::ifstream file(name);
        if (!file.is_open()) 
            throw std::runtime_error("Arquivo nao encontrado: " + name);

        if (!(file >> nVertices >> nEdges)) {
            throw std::runtime_error("Erro ao ler cabecalho da instancia.");
        }

        edges.reserve(nEdges);
        int u, v;
        for (int i = 0; i < nEdges; ++i) {
            if (file >> u >> v) {
                edges.push_back({u, v});
            }
        }
        
        std::cout << "[DEBUG] VC Loaded: V=" << nVertices << ", E=" << nEdges << std::endl;
    }

    void decode(rkolib::core::TSol &s) const override {
        // 1. Redimensionar/Limpar objetivos conforme o padrão da biblioteca
        if (s.objs.size() != 1) {
            s.objs.assign(1, 0.0);
        } else {
            s.objs[0] = 0.0;
        }

        int selectedCount = 0;
        // Vetor auxiliar para checagem rápida de cobertura
        std::vector<bool> inCover(nVertices, false);

        // 2. Decodificação das chaves
        for (int i = 0; i < nVertices; ++i) {
            if (s.rk[i] > 0.5) {
                inCover[i] = true;
                selectedCount++;
            }
        }

        // 3. Contagem de arestas não cobertas
        int uncoveredEdges = 0;
        for (const auto &edge : edges) {
            if (!inCover[edge.first] && !inCover[edge.second]) {
                uncoveredEdges++;
            }
        }

        // 4. OFV com Penalidade
        // Se houver arestas não cobertas, a solução é "pior" que qualquer solução válida
        double penalty = (double)uncoveredEdges * (nVertices + 1);
        s.objs[0] = (double)selectedCount + penalty;
        s.ofv = s.objs[0];
    }

    int getDimension() const override { return nVertices; }
    int getNumObjectives() const override { return 1; }
};

// --- Exportação do Plugin ---
extern "C" {
    #ifdef _WIN32
        __declspec(dllexport) rkolib::core::IProblem *create_problem() {
            return new VertexCoverProblem();
        }
        __declspec(dllexport) void destroy_problem(rkolib::core::IProblem *p) {
            delete p;
        }
    #else
        __attribute__((visibility("default"))) rkolib::core::IProblem *create_problem() {
            return new VertexCoverProblem();
        }
        __attribute__((visibility("default"))) void destroy_problem(rkolib::core::IProblem *p) {
            delete p;
        }
    #endif
}