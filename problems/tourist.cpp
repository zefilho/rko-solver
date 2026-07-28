#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <random>

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

struct No {
  int id;
  double x, y;
  double duracao; 
  double premio_q; 
};

class TouristProblem : public rkolib::core::IProblem {
private:
  int nObj;
  int n;
  double M, w0;
  int T_dias;

  std::vector<No> nos;
  std::vector<std::vector<double>> a, b, P, C, E;
  std::vector<int> rt, st;
  std::vector<std::vector<int>> R, H;

  void pularAte(std::ifstream& file, const std::string& marcador) {
    std::string linha;
    while (std::getline(file, linha)) {
      if (linha.find(marcador) != std::string::npos) return;
    }
    throw std::runtime_error("Marcador nao encontrado: " + marcador);
  }

  std::string valorApos(const std::string& token) {
    size_t pos = token.find('=');
    if (pos == std::string::npos) return "0";
    return token.substr(pos + 1);
  }

  std::vector<std::vector<double>> lerMatrizDouble(std::ifstream& file, int linhas, int colunas) {
    std::vector<std::vector<double>> mat(linhas, std::vector<double>(colunas, 0.0));
    for (int i = 0; i < linhas; ++i) {
      for (int j = 0; j < colunas; ++j) {
        if (!(file >> mat[i][j])) mat[i][j] = 0.0; // Prevenção de EOF
      }
    }
    return mat;
  }

public:
  TouristProblem() : nObj(2), n(0), M(0.0), w0(0.0), T_dias(0) {}
  ~TouristProblem() override = default;

  void load(const std::string &nomeArquivo) override {
    std::ifstream file(nomeArquivo);
    if (!file.is_open()) throw std::runtime_error("Erro abrir: " + nomeArquivo);

    std::string token, linha;

    pularAte(file, "PARAMETROS_GERAIS");
    file >> token; n = std::stoi(valorApos(token));
    file >> token; M = std::stod(valorApos(token));
    file >> token; w0 = std::stod(valorApos(token));
    file >> token; T_dias = std::stoi(valorApos(token));

    // VALIDAÇÃO CRÍTICA DE LIMITES
    if (n <= 0) n = 1;
    if (T_dias <= 0) T_dias = 1;

    pularAte(file, "LIMITES_ATRACOES_POR_DIA");
    rt.resize(T_dias, 0);
    st.resize(T_dias, 0);
    for (int t = 0; t < T_dias; ++t) {
      int dia = 0, rmin = 0, smax = 0;
      file >> dia >> rmin >> smax;
      
      // VACINA 1: Impede Segfault de Alocação Gigante ou Negativa
      rt[t] = std::max(0, rmin);
      st[t] = std::max(0, std::min(smax, n)); 
    }

    pularAte(file, "TABELA_NOS");
    nos.resize(n);
    for (int i = 0; i < n; ++i) {
      No no;
      if (file >> no.id >> no.x >> no.y >> no.duracao >> no.premio_q) nos[i] = no;
    }

    pularAte(file, "JANELAS_TEMPO_ABERTURA_A"); a = lerMatrizDouble(file, n, T_dias);
    pularAte(file, "JANELAS_TEMPO_FECHAMENTO_B"); b = lerMatrizDouble(file, n, T_dias);
    pularAte(file, "MATRIZ_PREMIOS_ORDEM_P"); P = lerMatrizDouble(file, n, n);
    pularAte(file, "MATRIZ_DISTANCIAS_C"); C = lerMatrizDouble(file, n, n);
    pularAte(file, "MATRIZ_TEMPO_VIAGEM_E"); E = lerMatrizDouble(file, n, n);

    pularAte(file, "RESTAURANTES_R");
    R.resize(T_dias);
    for (int t = 0; t < T_dias; ++t) {
      std::getline(file, linha);
      std::istringstream ss(linha);
      std::string prefixo; ss >> prefixo; 
      int id;
      while (ss >> id) {
        int idx = id - 1;
        if (idx >= 0 && idx < n) R[t].push_back(idx); 
      }
    }

    pularAte(file, "HOTEIS_H");
    H.resize(T_dias > 1 ? T_dias - 1 : 1);
    for (int t = 0; t < T_dias - 1; ++t) {
      std::getline(file, linha);
      std::istringstream ss(linha);
      std::string prefixo; ss >> prefixo; 
      int id;
      while (ss >> id) {
        int idx = id - 1;
        if (idx >= 0 && idx < n) H[t].push_back(idx); 
      }
    }
  }

  // =======================================================
  // MÉTODO AUXILIAR: Simula a rota de um dia e verifica viabilidade temporal
  // Retorna true se a rota é viável; preenche z1 e z2 parciais.
  // =======================================================
  struct SimResult {
    bool viavel;
    double z1_parcial;
    double z2_parcial;
    int visitas;
    int no_final;   // nó onde o turista termina o dia (hotel ou base)
  };

  SimResult simulateRoute(const std::vector<int> &rota_dia, int t,
                          int no_partida, int restaurante, int hotel) const {
    SimResult res = {true, 0.0, 0.0, 0, no_partida};
    double tempo = w0;
    int no_atual = no_partida;

    // Visitar atrações na ordem da rota
    for (int k = 0; k < static_cast<int>(rota_dia.size()); ++k) {
      int atr = rota_dia[k];
      if (atr < 0 || atr >= n) continue;

      double tempo_chegada = tempo + E[no_atual][atr];
      tempo_chegada = std::max(tempo_chegada, a[atr][t]);

      if (tempo_chegada > b[atr][t]) {
        res.viavel = false;
        return res;
      }

      res.z2_parcial += C[no_atual][atr];
      int k_safe = std::min(k, n - 1);
      res.z1_parcial += nos[atr].premio_q + P[k_safe][atr];
      tempo = tempo_chegada + nos[atr].duracao;
      no_atual = atr;
      res.visitas++;
    }

    // Deslocamento ao restaurante
    res.z2_parcial += C[no_atual][restaurante];
    tempo += E[no_atual][restaurante] + nos[restaurante].duracao;
    no_atual = restaurante;

    // Deslocamento ao hotel
    if (hotel >= 0 && hotel < n) {
      res.z2_parcial += C[no_atual][hotel];
      no_atual = hotel;
    }

    res.no_final = no_atual;
    return res;
  }

  // =======================================================
  // MÉTODO DECODE (Otimizado com Inserção Híbrida)
  // =======================================================
  void decode(rkolib::core::TSol &s) const override {
    if (s.rk.size() < static_cast<size_t>(getDimension())) return;

    auto clamp_key = [](double k) {
        if (std::isnan(k) || std::isinf(k)) return 0.0;
        return std::max(0.0, std::min(k, 0.999999));
    };

    // --- Layout de Segmentos ---
    // Segmento A (Prioridades):       s.rk[0        .. n-1]
    // Segmento B (Fator RCL):         s.rk[n]                  -> 1 ÚNICA CHAVE
    // Segmento C (Restaurantes):      s.rk[n+1      .. n+T]
    // Segmento D (Hotéis):            s.rk[n+T+1    .. n+2T-1]
    int offset_B = n;
    int offset_C = n + 1;
    int offset_D = n + 1 + T_dias;

    // 1. Segmento A: Ordenar atrações por prioridade (crescente de chave)
    std::vector<std::pair<double, int>> prioridade(n);
    for (int i = 0; i < n; ++i) {
      prioridade[i] = {clamp_key(s.rk[i]), i};
    }
    std::sort(prioridade.begin(), prioridade.end());

    // 2. Semente determinística para o modo aleatório
    unsigned int semente = 0;
    for (int i = 0; i < static_cast<int>(s.rk.size()); ++i) {
      semente ^= static_cast<unsigned int>(s.rk[i] * 1e6) + static_cast<unsigned int>(i);
    }
    std::mt19937 local_rng(semente);

    // 3. Pré-selecionar restaurantes e hotéis para cada dia (Segmentos C e D)
    std::vector<int> restaurantes(T_dias, 0);
    std::vector<int> hoteis(T_dias, 0);

    for (int t = 0; t < T_dias; ++t) {
      if (!R[t].empty()) {
        double ck = clamp_key(s.rk[offset_C + t]);
        int idx = static_cast<int>(ck * R[t].size());
        idx = std::max(0, std::min(idx, static_cast<int>(R[t].size()) - 1));
        restaurantes[t] = R[t][idx];
      }
      if (restaurantes[t] < 0 || restaurantes[t] >= n) restaurantes[t] = 0;

      if (t < T_dias - 1 && !H[t].empty()) {
        double ck = clamp_key(s.rk[offset_D + t]);
        int idx = static_cast<int>(ck * H[t].size());
        idx = std::max(0, std::min(idx, static_cast<int>(H[t].size()) - 1));
        hoteis[t] = H[t][idx];
      } else {
        hoteis[t] = 0;
      }
      if (hoteis[t] < 0 || hoteis[t] >= n) hoteis[t] = 0;
    }

    // 4. Construção iterativa das rotas diárias com inserção híbrida
    std::vector<bool> visitada(n, false);
    std::vector<std::vector<int>> rotas(T_dias);
    double z1_qualidade = 0.0;
    double z2_distancia = 0.0;
    int no_atual = 0; // Nó de partida global (aeroporto/base)

    // Estrutura auxiliar para ordenar as posições viáveis pelo custo
    struct PosCusto {
      int pos;
      double custo;
      bool operator<(const PosCusto& outro) const {
        return custo < outro.custo;
      }
    };

    for (int t = 0; t < T_dias; ++t) {
      int visitas_dia = 0;
      std::vector<PosCusto> posicoes_viaveis;
      posicoes_viaveis.reserve(n);

      // Tentar inserir cada atração na rota do dia (por ordem de prioridade)
      for (const auto &[chave, id_atr] : prioridade) {
        if (visitada[id_atr]) continue;
        if (visitas_dia >= st[t]) break;

        if (a[id_atr][t] > b[id_atr][t]) continue;

        posicoes_viaveis.clear();
        const int limite_posicoes = static_cast<int>(rotas[t].size());

        // Avalia TODAS as posições de inserção e calcula o custo
        for (int pos = 0; pos <= limite_posicoes; ++pos) {
          rotas[t].insert(rotas[t].begin() + pos, id_atr);
          SimResult sim = simulateRoute(rotas[t], t, no_atual,
                                        restaurantes[t], hoteis[t]);

          if (sim.viavel) {
            // Custo de inserção: quanto menor, melhor.
            double custo = sim.z2_parcial - sim.z1_parcial; 
            posicoes_viaveis.push_back({pos, custo});
          }

          rotas[t].erase(rotas[t].begin() + pos);
        }

        int pos_final = -1;
        
        // --- NOVA LÓGICA: Lista Restrita de Candidatos (RCL) ---
        if (!posicoes_viaveis.empty()) {
          // 1. Ordena os candidatos do melhor (menor custo) para o pior
          std::sort(posicoes_viaveis.begin(), posicoes_viaveis.end());

          // 2. Define o tamanho da lista baseado na única chave do Segmento B
          double fator_rcl = clamp_key(s.rk[offset_B]);
          
          // Tamanho da lista = ceil(fator * total_viaveis). Mínimo de 1 candidato.
          // Se fator_rcl for 0.0 -> rcl_size = 1 (Puramente Guloso)
          // Se fator_rcl for quase 1.0 -> rcl_size = todas viáveis (Puramente Aleatório)
          int rcl_size = std::max(1, static_cast<int>(std::ceil(fator_rcl * posicoes_viaveis.size())));
          
          // Limita o tamanho máximo por segurança
          rcl_size = std::min(rcl_size, static_cast<int>(posicoes_viaveis.size()));

          // 3. Sorteia um candidato APENAS dentre os melhores da lista restrita
          std::uniform_int_distribution<int> dist(0, rcl_size - 1);
          pos_final = posicoes_viaveis[dist(local_rng)].pos;
        }

        // Insere na posição sorteada da RCL
        if (pos_final >= 0) {
          rotas[t].insert(rotas[t].begin() + pos_final, id_atr);
          visitada[id_atr] = true;
          visitas_dia++;
        }
      }

      // 5. Avaliar a rota final do dia e acumular objetivos
      SimResult resultado = simulateRoute(rotas[t], t, no_atual,
                                          restaurantes[t], hoteis[t]);
      z1_qualidade += resultado.z1_parcial;
      z2_distancia += resultado.z2_parcial;

      if (resultado.visitas < rt[t]) z1_qualidade -= M;

      no_atual = resultado.no_final;
    }

    // 6. Gravar objetivos
    s.objs.assign(nObj, 0.0);
    // Objetivo 1: Qualidade (maximizar -> negativo para minimização do motor)
    s.objs[0] = -z1_qualidade;
    // Objetivo 2: Distância (minimizar -> positivo)
    s.objs[1] = z2_distancia;
  }

  // Dimensão reduzida: n + 3T - 1 (era 2n + 2T - 1)
  int getDimension() const override { return n + (3 * T_dias) - 1; }
  int getNumObjectives() const override { return nObj; }
};

extern "C" {
  #ifdef _WIN32
    __declspec(dllexport) rkolib::core::IProblem *create_problem() { return new TouristProblem(); }
    __declspec(dllexport) void destroy_problem(rkolib::core::IProblem *p) { delete p; }
  #else
    __attribute__((visibility("default"))) rkolib::core::IProblem *create_problem() { return new TouristProblem(); }
    __attribute__((visibility("default"))) void destroy_problem(rkolib::core::IProblem *p) { delete p; }
  #endif
}