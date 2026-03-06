#include "rkolib/core/solver.hpp"

using namespace rkolib::core;

class KnapsackProblem : public IProblem {
private:
  int nItems, nObj;
  double cap;
  std::vector<double> w;
  std::vector<std::vector<double>> objectives; // Matriz de valores
  std::vector<double> b;                       // Apenas para Mono

public:
  // Singleton Boilerplate
  KnapsackProblem() : nItems(0), nObj(1), cap(0.0) {}

  ~KnapsackProblem() override = default;

  void load(const std::string &name) override {
    std::ifstream file(name);
    if (!file.is_open())
      throw std::runtime_error("File not found: " + name);

    // 1. LEITURA SEGURA DO CABEÇALHO
    std::string line;
    std::getline(file, line); // Lê a primeira linha inteira
    std::stringstream ss(line);

    ss >> nItems >> cap;

    // Tenta ler o terceiro número. Se falhar, é 1.
    if (!(ss >> nObj)) {
      nObj = 1;
    }

    std::cout << "[DEBUG] Loading: nItems=" << nItems << ", Cap=" << cap
              << ", nObj=" << nObj << std::endl;

    w.resize(nItems);

    // 2. LEITURA DO CORPO
    if (nObj > 1) {
      // --- MODO MULTI ---
      objectives.assign(nObj, std::vector<double>(nItems));

      for (int i = 0; i < nItems; i++) {
        // Formato esperado: PESO OBJ1 OBJ2 ...
        file >> w[i];
        for (int k = 0; k < nObj; k++) {
          file >> objectives[k][i];
        }

        // Debug dos primeiros itens para garantir
        if (i < 3) {
          std::cout << "[DEBUG] Item " << i << ": W=" << w[i]
                    << " Val1=" << objectives[0][i] << std::endl;
        }
      }
    } else {
      // --- MODO MONO ---
      b.resize(nItems);
      for (int i = 0; i < nItems; i++) {
        // Formato esperado: VALOR PESO
        file >> b[i] >> w[i];

        if (i < 3) {
          std::cout << "[DEBUG] Item " << i << ": Val=" << b[i] << " W=" << w[i]
                    << std::endl;
        }
      }
    }
  }

  // void decode(TSol &s) const override {
  //     double totalW = 0.0;

  //     // Garante tamanho do vetor
  //     if (s.objs.size() != (size_t)nObj) s.objs.assign(nObj, 0.0);
  //     else std::fill(s.objs.begin(), s.objs.end(), 0.0);

  //     for (int i = 0; i < nItems; i++) {
  //         if (s.rk[i] > 0.5) {
  //             totalW += w[i];
  //             if (nObj > 1) {
  //                 for (int k = 0; k < nObj; k++) s.objs[k] +=
  //                 objectives[k][i];
  //             } else {
  //                 s.objs[0] += b[i];
  //             }
  //         }
  //     }

  //     // Penalidade por Capacidade
  //     double infeasible = std::max(0.0, totalW - cap);
  //     if (infeasible > 0) {
  //         s.ofv = 1e15 + (1000.0 * infeasible); // Valor proibitivo
  //         return;
  //     }

  //     // Se Mono, preenchemos o OFV final aqui (Minimização do negativo do
  //     lucro) if (nObj == 1) s.ofv = -s.objs[0];

  //     // Se Multi, deixamos s.ofv quieto (ou 0), o Solver vai sobrescrever.
  // }
  void decode(TSol &s) const override {
    // 1. Cria pares (RandomKey, IndiceOriginal) para ordenar
    std::vector<std::pair<double, int>> items(nItems);
    for (int i = 0; i < nItems; ++i) {
      items[i] = {s.rk[i], i};
    }

    // 2. Ordena decrescente: Quem tem maior RK tem prioridade
    std::sort(items.rbegin(), items.rend());

    double currentW = 0.0;

    // Zera/Redimensiona os objetivos
    if (s.objs.size() != (size_t)nObj)
      s.objs.assign(nObj, 0.0);
    else
      std::fill(s.objs.begin(), s.objs.end(), 0.0);

    // 3. Decodificador Guloso: Enche a mochila com prioridade
    for (const auto &item : items) {
      int idx = item.second; // Índice real do item

      // Se cabe, coloca!
      if (currentW + w[idx] <= cap) {
        currentW += w[idx];

        // Soma os objetivos
        if (nObj > 1) {
          for (int k = 0; k < nObj; k++) {
            s.objs[k] += objectives[k][idx];
          }
        } else {
          s.objs[0] += b[idx];
        }
      }
    }

    // 4. Finalização
    // Como usamos o método guloso, a solução é SEMPRE válida.
    // Removemos qualquer lógica de penalidade (1e15).

    if (nObj == 1) {
      // Para mono-objetivo (maximização), usamos negativo para minimizar
      s.ofv = -s.objs[0];
    }

    // NOTA: Se nObj > 1, deixamos s.ofv como 0.0 aqui.
    // O RkoSolver calculará o valor escalarizado (Tchebycheff) logo em seguida.
  }

  int getDimension() const override { return nItems; }
  int getNumObjectives() const override { return nObj; }
};

extern "C" {

EXPORT_PLUGIN rkolib::core::IProblem *create_problem() {
  return new KnapsackProblem();
}

EXPORT_PLUGIN void destroy_problem(rkolib::core::IProblem *p) { delete p; }
}