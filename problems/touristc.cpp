// =====================================================================
// VARIAÇÃO 2 — Decoder "cluster-first, route-second" com reparo
//
// Ideia central: o decoder original preenche os dias em sequência e de forma
// gulosa, então o DIA 1 sequestra as melhores atrações e os dias finais ficam
// abaixo de rt[t], levando à penalidade M. Aqui o cromossomo decide
// EXPLICITAMENTE em que dia cada atração é candidata (clustering), e só depois
// resolve o roteamento dentro de cada dia. 
//
// Refatorado para utilizar exclusivamente a estratégia LOV (Largest Order Value)
// para a priorização de atrações.
//
// Layout do cromossomo:  2n + 2T - 1
//   A: rk[0        .. n-1]        prioridade das atrações (LOV)
//   B: rk[n        .. 2n-1]       dia preferido de cada atração
//   C: rk[2n       .. 2n+T-1]     restaurantes
//   D: rk[2n+T     .. 2n+2T-2]    hotéis
// =====================================================================

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <functional>

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
  virtual void setDebugMode(int debug) { (void)debug; }
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
  int debug_mode = 0;
  int nObj;
  int n;
  double M, w0;
  int T_dias;

  double escala_C = 1.0;
  double escala_Q = 1.0;

  std::vector<No> nos;
  std::vector<std::vector<double>> a, b, P, C, E;
  std::vector<int> rt, st;
  std::vector<std::vector<int>> R, H;

  void pularAte(std::ifstream &file, const std::string &marcador) {
    std::string linha;
    while (std::getline(file, linha)) {
      if (linha.find(marcador) != std::string::npos) return;
    }
    throw std::runtime_error("Marcador nao encontrado: " + marcador);
  }

  std::string valorApos(const std::string &token) {
    size_t pos = token.find('=');
    if (pos == std::string::npos) return "0";
    return token.substr(pos + 1);
  }

  std::vector<std::vector<double>> lerMatrizDouble(std::ifstream &file, int linhas, int colunas) {
    std::vector<std::vector<double>> mat(linhas, std::vector<double>(colunas, 0.0));
    for (int i = 0; i < linhas; ++i)
      for (int j = 0; j < colunas; ++j)
        if (!(file >> mat[i][j])) mat[i][j] = 0.0;
    return mat;
  }

public:
  TouristProblem() : debug_mode(0), nObj(2), n(0), M(0.0), w0(0.0), T_dias(0) {}
  ~TouristProblem() override = default;

  void setDebugMode(int debug) override { debug_mode = debug; }

  void load(const std::string &nomeArquivo) override {
    std::ifstream file(nomeArquivo);
    if (!file.is_open()) {
      if (debug_mode > 0)
        std::cerr << "[ERROR][tourist] Falha ao abrir arquivo: " << nomeArquivo << std::endl;
      throw std::runtime_error("Erro abrir: " + nomeArquivo);
    }

    std::string token, linha;

    pularAte(file, "PARAMETROS_GERAIS");
    file >> token; n = std::stoi(valorApos(token));
    file >> token; M = std::stod(valorApos(token));
    file >> token; w0 = std::stod(valorApos(token));
    file >> token; T_dias = std::stoi(valorApos(token));

    if (n <= 0) n = 1;
    if (T_dias <= 0) T_dias = 1;

    pularAte(file, "LIMITES_ATRACOES_POR_DIA");
    rt.resize(T_dias, 0);
    st.resize(T_dias, 0);
    for (int t = 0; t < T_dias; ++t) {
      int dia = 0, rmin = 0, smax = 0;
      file >> dia >> rmin >> smax;
      rt[t] = std::max(0, rmin);
      st[t] = std::max(0, std::min(smax, n));
    }

    pularAte(file, "TABELA_NOS");
    nos.resize(n);
    for (int i = 0; i < n; ++i) {
      No no;
      if (file >> no.id >> no.x >> no.y >> no.duracao >> no.premio_q) nos[i] = no;
    }

    pularAte(file, "JANELAS_TEMPO_ABERTURA_A");   a = lerMatrizDouble(file, n, T_dias);
    pularAte(file, "JANELAS_TEMPO_FECHAMENTO_B"); b = lerMatrizDouble(file, n, T_dias);
    pularAte(file, "MATRIZ_PREMIOS_ORDEM_P");     P = lerMatrizDouble(file, n, n);
    pularAte(file, "MATRIZ_DISTANCIAS_C");        C = lerMatrizDouble(file, n, n);
    pularAte(file, "MATRIZ_TEMPO_VIAGEM_E");      E = lerMatrizDouble(file, n, n);

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

    double somaC = 0.0; int cnt = 0;
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j)
        if (i != j) { somaC += C[i][j]; ++cnt; }
    escala_C = (cnt > 0 && somaC > 0.0) ? somaC / cnt : 1.0;

    double somaQ = 0.0;
    for (int i = 0; i < n; ++i) somaQ += nos[i].premio_q;
    escala_Q = (somaQ > 0.0) ? somaQ / n : 1.0;
    if (!(escala_C > 0.0) || std::isnan(escala_C)) escala_C = 1.0;
    if (!(escala_Q > 0.0) || std::isnan(escala_Q)) escala_Q = 1.0;

    if (debug_mode > 0) {
      std::cout << "[DEBUG][tourist-v2] Instancia carregada: " << nomeArquivo
                << " | n=" << n << ", T_dias=" << T_dias
                << ", Dimensao=" << getDimension() << std::endl;
    }
  }

  struct SimResult {
    bool viavel;
    double z1_parcial;
    double z2_parcial;
    int visitas;
    int no_final;
  };

  SimResult simulateRoute(const std::vector<int> &rota_dia, int t,
                          int no_partida, int restaurante, int hotel) const {
    SimResult res = {true, 0.0, 0.0, 0, no_partida};
    double tempo = w0;
    int no_atual = no_partida;

    for (int k = 0; k < static_cast<int>(rota_dia.size()); ++k) {
      int atr = rota_dia[k];
      if (atr < 0 || atr >= n) continue;

      double tempo_chegada = std::max(tempo + E[no_atual][atr], a[atr][t]);
      if (tempo_chegada > b[atr][t]) { res.viavel = false; return res; }

      res.z2_parcial += C[no_atual][atr];
      int k_safe = std::min(k, n - 1);
      res.z1_parcial += nos[atr].premio_q + P[k_safe][atr];
      tempo = tempo_chegada + nos[atr].duracao;
      no_atual = atr;
      res.visitas++;
    }

    if (restaurante >= 0 && restaurante < n) {
      double chegada_r = std::max(tempo + E[no_atual][restaurante], a[restaurante][t]);
      if (chegada_r > b[restaurante][t]) { res.viavel = false; return res; }
      res.z2_parcial += C[no_atual][restaurante];
      tempo = chegada_r + nos[restaurante].duracao;
      no_atual = restaurante;
    }

    if (hotel >= 0 && hotel < n) {
      double chegada_h = std::max(tempo + E[no_atual][hotel], a[hotel][t]);
      if (chegada_h > b[hotel][t]) { res.viavel = false; return res; }
      res.z2_parcial += C[no_atual][hotel];
      no_atual = hotel;
    }

    res.no_final = no_atual;
    return res;
  }

private:
  bool melhorInsercao(std::vector<int> &rota, int t, int no_partida,
                      int restaurante, int hotel, int atr,
                      const SimResult &base, int &pos_melhor, double &custo_melhor) const {
    pos_melhor = -1;
    custo_melhor = std::numeric_limits<double>::infinity();
    const int limite = static_cast<int>(rota.size());

    for (int pos = 0; pos <= limite; ++pos) {
      rota.insert(rota.begin() + pos, atr);
      SimResult sim = simulateRoute(rota, t, no_partida, restaurante, hotel);
      if (sim.viavel) {
        double dz1 = sim.z1_parcial - base.z1_parcial;
        double dz2 = sim.z2_parcial - base.z2_parcial;
        double custo = (dz2 / escala_C) - (dz1 / escala_Q);
        if (custo < custo_melhor) { custo_melhor = custo; pos_melhor = pos; }
      }
      rota.erase(rota.begin() + pos);
    }
    return pos_melhor >= 0;
  }

public:
  void decode(rkolib::core::TSol &s) const override {
    if (s.rk.size() < static_cast<size_t>(getDimension())) {
      if (debug_mode > 0)
        std::cerr << "[ERROR][tourist-v2] Cromossomo (" << s.rk.size()
                  << ") menor que a dimensao (" << getDimension() << ")." << std::endl;
      return;
    }

    auto clamp_key = [](double k) {
      if (std::isnan(k) || std::isinf(k)) return 0.0;
      return std::max(0.0, std::min(k, 0.999999));
    };

    auto discretize_index = [&](double k, int max_val) {
      return std::min(std::max(0, static_cast<int>(clamp_key(k) * max_val)), max_val - 1);
    };

    const int off_dia   = n;
    const int off_rest  = 2 * n;
    const int off_hotel = 2 * n + T_dias;

    // --- Fase 1: clustering (atração -> dia candidato) ---
    std::vector<int> dia_pref(n, 0);
    for (int i = 0; i < n; ++i) {
      dia_pref[i] = discretize_index(s.rk[off_dia + i], T_dias);
    }

    // --- Fase 1.5: Geração da ordem de prioridade via Largest Order Value (LOV) ---
    std::vector<std::pair<double, int>> avaliacao(n);
    for (int i = 0; i < n; ++i) {
      avaliacao[i] = {clamp_key(s.rk[i]), i};
    }
    
    // LOV: ordena os valores contínuos em ordem decrescente 
    std::ranges::sort(avaliacao, std::greater<>{});

    std::vector<int> lov_sequence(n);
    for (int rank = 0; rank < n; ++rank) {
      int id_original = avaliacao[rank].second;
      lov_sequence[id_original] = rank + 1; // Rank 1-based conforme literatura
    }

    // Para iterar pela prioridade (do rank 1 até n), derivamos ordem_atr
    std::vector<int> ordem_atr(n);
    for (int i = 0; i < n; ++i) {
      ordem_atr[i] = avaliacao[i].second; 
    }

    // Restaurantes e hotéis
    std::vector<int> restaurantes(T_dias, 0), hoteis(T_dias, -1);
    for (int t = 0; t < T_dias; ++t) {
      if (!R[t].empty()) {
        restaurantes[t] = R[t][discretize_index(s.rk[off_rest + t], static_cast<int>(R[t].size()))];
      }
      if (restaurantes[t] < 0 || restaurantes[t] >= n) restaurantes[t] = 0;

      if (t < T_dias - 1 && !H[t].empty()) {
        hoteis[t] = H[t][discretize_index(s.rk[off_hotel + t], static_cast<int>(H[t].size()))];
        if (hoteis[t] < 0 || hoteis[t] >= n) hoteis[t] = -1;
      } else {
        hoteis[t] = -1;
      }
    }

    std::vector<bool> visitada(n, false);
    std::vector<std::vector<int>> rotas(T_dias);
    double z1_qualidade = 0.0, z2_distancia = 0.0;
    int no_atual = 0;

    for (int t = 0; t < T_dias; ++t) {
      int visitas_dia = 0;
      SimResult base = simulateRoute(rotas[t], t, no_atual, restaurantes[t], hoteis[t]);

      // --- Fase 2: roteamento dentro do cluster do dia t ---
      for (int id_atr : ordem_atr) {
        if (id_atr < 0 || id_atr >= n) continue;
        if (visitada[id_atr] || dia_pref[id_atr] != t) continue;
        if (visitas_dia >= st[t]) break;
        if (a[id_atr][t] > b[id_atr][t]) continue;

        int pos; double custo;
        if (!melhorInsercao(rotas[t], t, no_atual, restaurantes[t], hoteis[t],
                            id_atr, base, pos, custo))
          continue;

        if (custo >= 0.0 && visitas_dia >= rt[t]) continue;

        rotas[t].insert(rotas[t].begin() + pos, id_atr);
        visitada[id_atr] = true;
        visitas_dia++;
        base = simulateRoute(rotas[t], t, no_atual, restaurantes[t], hoteis[t]);
      }

      // --- Fase 3a: reparo do mínimo rt[t] usando o pool global ---
      while (visitas_dia < rt[t] && visitas_dia < st[t]) {
        int melhor_atr = -1, melhor_pos = -1;
        double melhor_custo = std::numeric_limits<double>::infinity();

        for (int id_atr : ordem_atr) {
          if (id_atr < 0 || id_atr >= n) continue;
          if (visitada[id_atr]) continue;
          if (a[id_atr][t] > b[id_atr][t]) continue;

          int pos; double custo;
          if (melhorInsercao(rotas[t], t, no_atual, restaurantes[t], hoteis[t],
                             id_atr, base, pos, custo) && custo < melhor_custo) {
            melhor_custo = custo; melhor_atr = id_atr; melhor_pos = pos;
          }
        }

        if (melhor_atr < 0) break; 
        rotas[t].insert(rotas[t].begin() + melhor_pos, melhor_atr);
        visitada[melhor_atr] = true;
        visitas_dia++;
        base = simulateRoute(rotas[t], t, no_atual, restaurantes[t], hoteis[t]);
      }

      // --- Fase 3b: aproveita folga do dia com sobras do pool global ---
      bool melhorou = true;
      while (melhorou && visitas_dia < st[t]) {
        melhorou = false;
        int melhor_atr = -1, melhor_pos = -1;
        double melhor_custo = 0.0; 

        for (int id_atr : ordem_atr) {
          if (id_atr < 0 || id_atr >= n) continue;
          if (visitada[id_atr]) continue;
          if (a[id_atr][t] > b[id_atr][t]) continue;

          int pos; double custo;
          if (melhorInsercao(rotas[t], t, no_atual, restaurantes[t], hoteis[t],
                             id_atr, base, pos, custo) && custo < melhor_custo) {
            melhor_custo = custo; melhor_atr = id_atr; melhor_pos = pos;
          }
        }

        if (melhor_atr >= 0) {
          rotas[t].insert(rotas[t].begin() + melhor_pos, melhor_atr);
          visitada[melhor_atr] = true;
          visitas_dia++;
          base = simulateRoute(rotas[t], t, no_atual, restaurantes[t], hoteis[t]);
          melhorou = true;
        }
      }

      SimResult resultado = simulateRoute(rotas[t], t, no_atual, restaurantes[t], hoteis[t]);
      z1_qualidade += resultado.z1_parcial;
      z2_distancia += resultado.z2_parcial;

      if (resultado.visitas < rt[t]) {
        if (debug_mode > 0)
          std::cout << "[WARNING][tourist-v2] Dia " << (t + 1) << ": visitas="
                    << resultado.visitas << " < rt=" << rt[t] << std::endl;
        z1_qualidade -= M * (rt[t] - resultado.visitas);
      }

      no_atual = resultado.no_final;
    }

    s.objs.assign(nObj, 0.0);
    s.objs[0] = -z1_qualidade;
    s.objs[1] = z2_distancia;

    if (debug_mode > 0)
      std::cout << "[DEBUG][tourist-v2] Obj1(Qualidade)=" << -s.objs[0]
                << ", Obj2(Distancia)=" << s.objs[1] << std::endl;
  }

  int getDimension() const override {
    return (2 * n) + (2 * T_dias) - 1;
  }
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