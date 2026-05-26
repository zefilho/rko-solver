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

// ---------------------------------------------------------
// ESTRUTURA DO NÓ
// ---------------------------------------------------------
struct No {
  int id;
  double x, y;
  double duracao;  // d_i
  double premio_q; // q_i
};

// ---------------------------------------------------------
// CLASSE DO PROBLEMA
// ---------------------------------------------------------

class TouristProblem : public rkolib::core::IProblem {
private:
  int nObj;
  
  // Parâmetros da Instância
  int n;
  double M, w0;
  int T_dias;

  std::vector<No> nos;
  std::vector<std::vector<double>> a; // janelas de abertura  a[i][t]
  std::vector<std::vector<double>> b; // janelas de fechamento b[i][t]
  std::vector<int> rt;                // mínimo de atrações por dia
  std::vector<int> st;                // máximo de atrações por dia
  std::vector<std::vector<double>> P; // prêmios por ordem   P[k][i]
  std::vector<std::vector<double>> C; // distâncias          C[i][j]
  std::vector<std::vector<double>> E; // tempos de viagem    E[i][j]
  std::vector<std::vector<int>> R;    // restaurantes por dia R[t]
  std::vector<std::vector<int>> H;    // hotéis por dia       H[t]

  // ---------------------------------------------------------
  // UTILITÁRIOS DE LEITURA (Privados)
  // ---------------------------------------------------------
  void pularAte(std::ifstream& file, const std::string& marcador) {
    std::string linha;
    while (std::getline(file, linha)) {
      if (linha.find(marcador) != std::string::npos)
        return;
    }
    throw std::runtime_error("Marcador nao encontrado no arquivo: " + marcador);
  }

  std::string valorApos(const std::string& token) {
    size_t pos = token.find('=');
    if (pos == std::string::npos)
      throw std::runtime_error("Token invalido, esperado '=': " + token);
    return token.substr(pos + 1);
  }

  std::vector<std::vector<double>> lerMatrizDouble(std::ifstream& file, int linhas, int colunas) {
    std::vector<std::vector<double>> mat(linhas, std::vector<double>(colunas));
    for (int i = 0; i < linhas; ++i) {
      for (int j = 0; j < colunas; ++j) {
        file >> mat[i][j];
      }
    }
    return mat;
  }

public:
  // Inicializa o problema como bi-objetivo (nObj = 2)
  TouristProblem() : nObj(2), n(0), M(0.0), w0(0.0), T_dias(0) {}

  ~TouristProblem() override = default;

  // =======================================================
  // MÉTODO LOAD
  // =======================================================
  void load(const std::string &nomeArquivo) override {
    std::ifstream file(nomeArquivo);
    if (!file.is_open()) {
      throw std::runtime_error("Erro ao abrir o arquivo da instancia: " + nomeArquivo);
    }

    std::string token, linha;

    // 0. PARAMETROS_GERAIS
    pularAte(file, "PARAMETROS_GERAIS");
    file >> token; n = std::stoi(valorApos(token));
    file >> token; M = std::stod(valorApos(token));
    file >> token; w0 = std::stod(valorApos(token));
    file >> token; T_dias = std::stoi(valorApos(token));

    // 1. LIMITES_ATRACOES_POR_DIA
    pularAte(file, "LIMITES_ATRACOES_POR_DIA");
    rt.resize(T_dias);
    st.resize(T_dias);
    for (int t = 0; t < T_dias; ++t) {
      int dia, rmin, smax;
      file >> dia >> rmin >> smax;
      rt[t] = rmin;
      st[t] = smax;
    }

    // 2. TABELA_NOS
    pularAte(file, "TABELA_NOS");
    nos.resize(n);
    for (int i = 0; i < n; ++i) {
      No no;
      file >> no.id >> no.x >> no.y >> no.duracao >> no.premio_q;
      nos[i] = no;
    }

    // 3 e 4. JANELAS DE TEMPO (A e B)
    pularAte(file, "JANELAS_TEMPO_ABERTURA_A");
    a = lerMatrizDouble(file, n, T_dias);

    pularAte(file, "JANELAS_TEMPO_FECHAMENTO_B");
    b = lerMatrizDouble(file, n, T_dias);

    // 5, 6 e 7. MATRIZES P, C e E
    pularAte(file, "MATRIZ_PREMIOS_ORDEM_P");
    P = lerMatrizDouble(file, n, n);

    pularAte(file, "MATRIZ_DISTANCIAS_C");
    C = lerMatrizDouble(file, n, n);

    pularAte(file, "MATRIZ_TEMPO_VIAGEM_E");
    E = lerMatrizDouble(file, n, n);

    // 8. RESTAURANTES_R
    pularAte(file, "RESTAURANTES_R");
    R.resize(T_dias);
    for (int t = 0; t < T_dias; ++t) {
      std::getline(file, linha);
      std::istringstream ss(linha);
      std::string prefixo;
      ss >> prefixo; 
      int id;
      while (ss >> id) {
        int idx = id - 1;
        if (idx >= 0 && idx < n) {
          R[t].push_back(idx); 
        } else {
          std::cerr << "[Aviso] ID de Restaurante fora do limite n: " << id << ". Ignorado.\n";
        }
      }
      if(R[t].empty()) {
          throw std::runtime_error("Nenhum restaurante valido para o dia " + std::to_string(t+1));
      }
    }

    // 9. HOTEIS_H
    pularAte(file, "HOTEIS_H");
    H.resize(T_dias - 1);
    for (int t = 0; t < T_dias - 1; ++t) {
      std::getline(file, linha);
      std::istringstream ss(linha);
      std::string prefixo;
      ss >> prefixo; 
      int id;
      while (ss >> id) {
        int idx = id - 1;
        if (idx >= 0 && idx < n) {
          H[t].push_back(idx); 
        } else {
          std::cerr << "[Aviso] ID de Hotel fora do limite n: " << id << ". Ignorado.\n";
        }
      }
    }
    
    std::cout << "[DEBUG] Instancia carregada com sucesso: n=" << n 
              << ", Dias=" << T_dias << std::endl;
  }

  // =======================================================
  // MÉTODO DECODE
  // =======================================================
  void decode(rkolib::core::TSol &s) const override {
    fprintf(stderr, "  [PLUGIN TRACE] Entrou no decode do Tourist.\n");
    if (s.rk.size() < static_cast<size_t>(getDimension())) {
        fprintf(stderr, "  [PLUGIN TRACE] Erro: Cromossomo RK menor que a Dimensao.\n");
        return;
    }

    auto clamp_key = [](double k) {
        if (std::isnan(k) || std::isinf(k)) return 0.0;
        if (k < 0.0) return 0.0;
        if (k > 0.999999) return 0.999999; // Evita que chave 1.0 arredonde pra fora do array
        return k;
    };

    int num_atracoes = n;
    double z1_qualidade = 0.0;
    double z2_distancia = 0.0;

    fprintf(stderr, "  [PLUGIN TRACE] Extraindo prioridades (Segmento A)...\n");
    std::vector<std::pair<double, int>> prioridade_atracoes(num_atracoes);
    for (int i = 0; i < num_atracoes; ++i) {
      prioridade_atracoes[i] = {clamp_key(s.rk[i]), i};
    }
    
    std::sort(prioridade_atracoes.begin(), prioridade_atracoes.end());

    fprintf(stderr, "  [PLUGIN TRACE] Alocando matriz de atracoes_por_dia...\n");
    std::vector<std::vector<int>> atracoes_por_dia(T_dias);
    std::vector<int> contador_dia(T_dias, 0);

    for (int t = 0; t < T_dias; ++t) {
        atracoes_por_dia[t].resize(st[t], -1); 
    }

    fprintf(stderr, "  [PLUGIN TRACE] Distribuindo atracoes (Segmento B)...\n");
    for (const auto& par : prioridade_atracoes) {
      int id_atracao = par.second;
      
      double chave_dia = clamp_key(s.rk[num_atracoes + id_atracao]);
      int dia_alocado = static_cast<int>(chave_dia * T_dias);
      dia_alocado = std::max(0, std::min(dia_alocado, T_dias - 1));
      
      if (contador_dia[dia_alocado] < st[dia_alocado]) {
        atracoes_por_dia[dia_alocado][contador_dia[dia_alocado]] = id_atracao;
        contador_dia[dia_alocado]++; 
      }
    }

    fprintf(stderr, "  [PLUGIN TRACE] Calculando rotas e avaliando (Segmentos C e D)...\n");
    int offset_C = 2 * num_atracoes;
    int offset_D = offset_C + T_dias;
    int no_atual = 0; 

    for (int t = 0; t < T_dias; ++t) {
      
      // 5.1 Seleção Segura do Restaurante
      int restaurante_dia = 0; // Fallback seguro (Aeroporto/Base)
      if (!R[t].empty()) {
          double chave_rest = clamp_key(s.rk[offset_C + t]);
          int idx_rest = static_cast<int>(chave_rest * R[t].size());
          idx_rest = std::max(0, std::min(idx_rest, static_cast<int>(R[t].size() - 1)));
          restaurante_dia = R[t][idx_rest];
      }

      // 5.2 Seleção Segura do Hotel
      int hotel_noite = -1;
      if (t < T_dias - 1 && !H[t].empty()) {
          double chave_hotel = clamp_key(s.rk[offset_D + t]);
          int idx_hotel = static_cast<int>(chave_hotel * H[t].size());
          idx_hotel = std::max(0, std::min(idx_hotel, static_cast<int>(H[t].size() - 1)));
          hotel_noite = H[t][idx_hotel];
      } else {
          hotel_noite = 0; 
      }

      // Verificação Paranóica (Se a instância tiver lixo, cai no nó 0)
      if (restaurante_dia < 0 || restaurante_dia >= n) restaurante_dia = 0;
      if (hotel_noite < 0 || hotel_noite >= n) hotel_noite = 0;

      double tempo_atual = w0; 
      int visitas_hoje = 0;

      // 5.3 Simulação de Viagens e Janelas de Tempo
      for (int k = 0; k < contador_dia[t]; ++k) {
        int proxima_atracao = atracoes_por_dia[t][k];
        
        // Proteção contra lixo de memória
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

      // Fechamento do Roteiro Diário
      z2_distancia += C[no_atual][restaurante_dia];
      tempo_atual += E[no_atual][restaurante_dia] + nos[restaurante_dia].duracao;
      no_atual = restaurante_dia;

      if (hotel_noite != -1) {
        z2_distancia += C[no_atual][hotel_noite];
        no_atual = hotel_noite;
      }
    }

    fprintf(stderr, "  [PLUGIN TRACE] Gravando resultados em s.objs...\n");
    if(s.objs.size() != static_cast<size_t>(nObj)) {
        fprintf(stderr, "  [PLUGIN TRACE] Erro critico: s.objs nao tem o tamanho de nObj!\n");
        return;
    }
    s.objs[0] = -z1_qualidade;
    s.objs[1] = z2_distancia;
    fprintf(stderr, "  [PLUGIN TRACE] Decode do Tourist concluido com sucesso.\n");
  }

  // =======================================================
  // INFORMAÇÕES DO PROBLEMA
  // =======================================================
  // Para que o core da RKOLib saiba o tamanho do cromossomo:
  // Tamanho = (Num_Atracoes) + (Num_Atracoes) + (Num_Dias) + (Num_Dias - 1)
  int getDimension() const override { return (2 * n) + T_dias + (T_dias - 1); }
  int getNumObjectives() const override { return nObj; }
};

// ---------------------------------------------------------
// EXPORTAÇÃO DO PLUGIN (Contrato C)
// ---------------------------------------------------------
extern "C" {
  #ifdef _WIN32
    __declspec(dllexport) rkolib::core::IProblem *create_problem() {
      return new TouristProblem();
    }
    __declspec(dllexport) void destroy_problem(rkolib::core::IProblem *p) {
      delete p;
    }
  #else
    __attribute__((visibility("default"))) rkolib::core::IProblem *
    create_problem() {
      return new TouristProblem();
    }
    __attribute__((visibility("default"))) void
    destroy_problem(rkolib::core::IProblem *p) {
      delete p;
    }
  #endif
}