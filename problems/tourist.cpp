#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>

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
  // MÉTODO DECODE
  // =======================================================
  void decode(rkolib::core::TSol &s) const override {
    if (s.rk.size() < static_cast<size_t>(getDimension())) return;

    std::vector<double> rk_local = s.rk; 

    auto clamp_key = [](double k) {
        if (std::isnan(k) || std::isinf(k)) return 0.0;
        return std::max(0.0, std::min(k, 0.999999));
    };

    int num_atracoes = n; 
    double z1_qualidade = 0.0;
    double z2_distancia = 0.0;

    std::vector<std::pair<double, int>> prioridade_atracoes(num_atracoes);
    for (int i = 0; i < num_atracoes; ++i) {
      prioridade_atracoes[i] = {clamp_key(rk_local[i]), i};
    }
    std::sort(prioridade_atracoes.begin(), prioridade_atracoes.end());

    std::vector<std::vector<int>> atracoes_por_dia(T_dias);
    std::vector<int> contador_dia(T_dias, 0); 
    
    for (int t = 0; t < T_dias; ++t) {
        atracoes_por_dia[t].resize(st[t], -1); 
    }

    for (const auto& par : prioridade_atracoes) {
      int id_atracao = par.second;
      double chave_dia = clamp_key(rk_local[num_atracoes + id_atracao]);
      int dia_alocado = static_cast<int>(chave_dia * T_dias);
      dia_alocado = std::max(0, std::min(dia_alocado, T_dias - 1));
      
      if (contador_dia[dia_alocado] < st[dia_alocado]) {
        atracoes_por_dia[dia_alocado][contador_dia[dia_alocado]] = id_atracao;
        contador_dia[dia_alocado]++; 
      }
    }

    int offset_C = 2 * num_atracoes;
    int offset_D = offset_C + T_dias;
    int no_atual = 0; 

    for (int t = 0; t < T_dias; ++t) {
      int restaurante_dia = 0; 
      if (!R[t].empty()) {
          double chave_rest = clamp_key(rk_local[offset_C + t]);
          int idx_rest = static_cast<int>(chave_rest * R[t].size());
          idx_rest = std::max(0, std::min(idx_rest, static_cast<int>(R[t].size() - 1)));
          restaurante_dia = R[t][idx_rest];
      }

      int hotel_noite = -1;
      if (t < T_dias - 1 && !H[t].empty()) {
          double chave_hotel = clamp_key(rk_local[offset_D + t]);
          int idx_hotel = static_cast<int>(chave_hotel * H[t].size());
          idx_hotel = std::max(0, std::min(idx_hotel, static_cast<int>(H[t].size() - 1)));
          hotel_noite = H[t][idx_hotel];
      } else {
          hotel_noite = 0; 
      }

      if (restaurante_dia < 0 || restaurante_dia >= n) restaurante_dia = 0;
      if (hotel_noite < 0 || hotel_noite >= n) hotel_noite = 0;

      double tempo_atual = w0; 
      int visitas_hoje = 0;

      for (int k = 0; k < contador_dia[t]; ++k) {
        int proxima_atracao = atracoes_por_dia[t][k];
        if (proxima_atracao < 0 || proxima_atracao >= n) continue; 
        
        double tempo_chegada = tempo_atual + E[no_atual][proxima_atracao];
        tempo_chegada = std::max(tempo_chegada, a[proxima_atracao][t]);

        if (tempo_chegada <= b[proxima_atracao][t]) {
          z2_distancia += C[no_atual][proxima_atracao];
          int k_safe = std::min(static_cast<int>(k), n - 1);
          z1_qualidade += nos[proxima_atracao].premio_q + P[k_safe][proxima_atracao];
          tempo_atual = tempo_chegada + nos[proxima_atracao].duracao;
          no_atual = proxima_atracao;
          visitas_hoje++;
        }
      }

      if (visitas_hoje < rt[t]) z1_qualidade -= M; 

      z2_distancia += C[no_atual][restaurante_dia];
      tempo_atual += E[no_atual][restaurante_dia] + nos[restaurante_dia].duracao;
      no_atual = restaurante_dia;

      if (hotel_noite != -1) {
        z2_distancia += C[no_atual][hotel_noite];
        no_atual = hotel_noite;
      }
    }

    s.objs.assign(nObj, 0.0); 

    // Objetivo 1: Qualidade (Queremos maximizar, então enviamos negativo para o motor minimizar)
    s.objs[0] = -z1_qualidade; 
    
    // Objetivo 2: Distância (Já queremos minimizar, enviamos positivo)
    s.objs[1] = z2_distancia;
    //s.ofv = -z1_qualidade + z2_distancia;
  }

  int getDimension() const override { return (2 * n) + T_dias + (T_dias - 1); }
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