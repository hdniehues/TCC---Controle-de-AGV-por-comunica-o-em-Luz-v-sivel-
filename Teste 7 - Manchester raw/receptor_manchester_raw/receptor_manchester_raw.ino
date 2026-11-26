#include <Arduino.h>
#include "soc/gpio_reg.h"

// --- PINO DE RECEPÇÃO VLC ---
#define RX_PIN 18

// --- PINOS DA PONTE H (L293DNE) NA ESP32-S3 ---
#define IN1 17
#define IN2 16
#define IN3 15
#define IN4 7
#define EN1 5   // PWM Motor A
#define EN2 6   // PWM Motor B

const int PWM_freq = 500;
const int PWM_res  = 8; // 0 a 255

// --- BUFFERS DO SDR MANCHESTER ---
const int MAX_BORDAS = 4000; 
uint32_t tempo_bordas[MAX_BORDAS];
uint8_t nivel_bordas[MAX_BORDAS];

// Tabela de Velocidades PWM (4 bits -> 8 níveis ativos de 32 a 255 + 0 parado)
const uint8_t TABELA_PWM[16] = {
  0,   // 0x0: Parado
  32,  // 0x1: Nível 1 (Mínimo de referência)
  180, // 0x2: Nível 2
  190, // 0x3: Nível 3
  200, // 0x4: Nível 4 (Padrão Modo Manual / Retas)
  215, // 0x5: Nível 5
  230, // 0x6: Nível 6 (Padrão Curvas)
  242, // 0x7: Nível 7
  255, // 0x8: Nível 8 (Potência Máxima)
  255, 255, 255, 255, 255, 255, 255
};

// --- FILA DE MOVIMENTOS E MÁQUINA DE ESTADOS ---
#define TAMANHO_FILA 128
uint8_t fila_movimentos[TAMANHO_FILA];
int fila_head = 0; 
int fila_tail = 0; 

bool emergencia_ativa   = false;
bool executando_passo   = false;
bool modo_seta_continua = false;
bool em_pausa_transicao = false;

uint32_t tempo_inicio_passo     = 0;
uint32_t tempo_duracao_alvo     = 0;
uint32_t tempo_restante_pausado = 0;
uint32_t tempo_inicio_pausa     = 0;

uint8_t dir_atual_salva = 0;
uint8_t pwm_atual_salvo = 0;
int contagem_passos     = 1;

// ====================================================================
// FUNÇÕES FÍSICAS DE CONTROLE DOS MOTORES DC (L293DNE)
// ====================================================================
void motorA_frente(int vel) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(EN1, vel);
}

void motorA_tras(int vel) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(EN1, vel);
}

void motorB_frente(int vel) {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(EN2, vel);
}

void motorB_tras(int vel) {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(EN2, vel);
}

void parar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(EN1, 0);
  ledcWrite(EN2, 0);
}

void frente(int vel) {
  motorA_frente(vel);
  motorB_frente(vel);
}

void tras(int vel) {
  motorA_tras(vel);
  motorB_tras(vel);
}

void esquerda(int vel) {
  motorA_tras(vel);
  motorB_frente(vel);
}

void direita(int vel) {
  motorA_frente(vel);
  motorB_tras(vel);
}

void acionarMotoresPorBits(uint8_t dir_bits, uint8_t pwm_val) {
  if (pwm_val == 0) {
    parar();
    return;
  }
  switch (dir_bits) {
    case 0b00: tras(pwm_val);     break; // 00 - Trás
    case 0b01: frente(pwm_val);   break; // 01 - Frente
    case 0b10: direita(pwm_val);  break; // 10 - Direita
    case 0b11: esquerda(pwm_val); break; // 11 - Esquerda
  }
}

const char* nomeDirecao(uint8_t dir_bits) {
  switch (dir_bits) {
    case 0b00: return "TRAS (T)";
    case 0b01: return "FRENTE (F)";
    case 0b10: return "DIREITA (D)";
    case 0b11: return "ESQUERDA (E)";
    default:   return "DESCONHECIDO";
  }
}

uint32_t decodificarTempoMs(uint8_t tempo_bits) {
  switch (tempo_bits) {
    case 0b00: return 0;    // 0s
    case 0b10: return 2000; // 2s
    case 0b01: return 4000; // 4s
    case 0b11: return 6000; // 6s
    default:   return 0;
  }
}

// ====================================================================
// TRATAMENTO DOS PACOTES E GERENCIAMENTO DA FILA DE MOVIMENTOS
// ====================================================================
void processarComandoApp(uint8_t* payload) {
  uint8_t header = payload[0];

  // 1. PARADA DE EMERGÊNCIA (0xEE)
  if (header == 0xEE) {
    parar(); // Corta os motores físicos imediatamente!
    if (!emergencia_ativa && executando_passo && !modo_seta_continua) {
      uint32_t decorrido = millis() - tempo_inicio_passo;
      tempo_restante_pausado = (decorrido < tempo_duracao_alvo) ? (tempo_duracao_alvo - decorrido) : 0;
    }
    emergencia_ativa = true;
    modo_seta_continua = false;
    Serial.println("\n[EMERGENCIA] !!! PARADA IMEDIATA (0xEE) !!! Motores travados em 0V.");
    return;
  }
  
  // 2. RETOMAR MOVIMENTO (0xDD)
  if (header == 0xDD) {
    if (emergencia_ativa) {
      emergencia_ativa = false;
      if (executando_passo && tempo_restante_pausado > 0) {
        tempo_inicio_passo = millis();
        tempo_duracao_alvo = tempo_restante_pausado;
        acionarMotoresPorBits(dir_atual_salva, pwm_atual_salvo); // Religa os motores físicos!
        Serial.printf("\n[SISTEMA] >>> LIBERADO (0xDD). Retomando %s por mais %lu ms...\n",
                      nomeDirecao(dir_atual_salva), tempo_duracao_alvo);
      } else {
        Serial.println("\n[SISTEMA] >>> LIBERADO (0xDD). Aguardando proximo item da fila...");
      }
    }
    return;
  }

  // 3. ROTA AUTOMÁTICA OU SEQUÊNCIA DIGITADA (0x01) -> ENTRA NA FILA
  if (header == 0x01) {
    int adicionados = 0;
    for (int i = 1; i < 32; i++) {
      if (payload[i] == 0x00) break; // 0x00 encerra a sequência
      fila_movimentos[fila_tail] = payload[i];
      fila_tail = (fila_tail + 1) % TAMANHO_FILA;
      adicionados++;
    }
    Serial.printf("\n[FILA] Pacote 0x01 recebido! +%d movimento(s) enfileirado(s).\n", adicionados);
    return;
  }

  // 4. MODO MANUAL DE SETAS CONTÍNUAS (0x02)
  if (header == 0x02) {
    if (emergencia_ativa) return;
    uint8_t cmd = payload[1];

    if (cmd == 0x00) {
      parar();
      modo_seta_continua = false;
      executando_passo = false;
      Serial.println("\n[MANUAL SETAS] Botao solto -> Motores PARADOS.");
    } else {
      uint8_t dir_bits = (cmd >> 6) & 0x03;
      uint8_t vel_idx  = cmd & 0x0F;
      uint8_t pwm_val  = TABELA_PWM[vel_idx];

      dir_atual_salva = dir_bits;
      pwm_atual_salvo = pwm_val;

      acionarMotoresPorBits(dir_bits, pwm_val);

      modo_seta_continua = true;
      executando_passo   = true;
      tempo_inicio_passo = millis();
      tempo_duracao_alvo = 1800; // Janela de 1,8s renovada a cada 1,5s pelo emissor

      Serial.printf("\n[MANUAL SETAS] Movendo %s | PWM: %d (Byte: 0x%02X)\n",
                    nomeDirecao(dir_bits), pwm_val, cmd);
    }
  }
}

void gerenciarMotores() {
  if (emergencia_ativa) return;

  uint32_t agora = millis();

  // Pausa curta de 80ms entre movimentos da fila para proteger a ponte H contra picos de reversão
  if (em_pausa_transicao) {
    if (agora - tempo_inicio_pausa >= 80) {
      em_pausa_transicao = false;
    } else {
      return;
    }
  }

  // Verifica se o tempo do movimento atual acabou
  if (executando_passo && (agora - tempo_inicio_passo >= tempo_duracao_alvo)) {
    parar(); // Desliga os motores físicos
    executando_passo = false;

    if (modo_seta_continua) {
      modo_seta_continua = false;
      Serial.println("[MANUAL SETAS] Timeout de seguranca -> Motores parados.");
    } else {
      em_pausa_transicao = true;
      tempo_inicio_pausa = agora;
      Serial.println("[AGV] Movimento concluido. Motores parados.");
    }
    return;
  }

  // Se estiver livre e houver passos na fila, puxa e aciona o próximo movimento
  if (!executando_passo && !modo_seta_continua && (fila_head != fila_tail)) {
    uint8_t byte_cmd = fila_movimentos[fila_head];
    fila_head = (fila_head + 1) % TAMANHO_FILA;

    // Decodifica [DD: 7..6] [TT: 5..4] [VVVV: 3..0]
    uint8_t dir_bits   = (byte_cmd >> 6) & 0x03;
    uint8_t tempo_bits = (byte_cmd >> 4) & 0x03;
    uint8_t vel_idx    = byte_cmd & 0x0F;

    tempo_duracao_alvo = decodificarTempoMs(tempo_bits);
    uint8_t pwm_val    = TABELA_PWM[vel_idx];

    dir_atual_salva = dir_bits;
    pwm_atual_salvo = pwm_val;

    // Aciona fisicamente a ponte H L293DNE
    acionarMotoresPorBits(dir_bits, pwm_val);

    tempo_inicio_passo = agora;
    executando_passo   = true;

    Serial.printf("\n[AGV] === PASSO %d EM EXECUCAO ===\n", contagem_passos++);
    Serial.printf("[AGV] Byte: 0x%02X | Dir: %s | Tempo: %lu ms | PWM: %d\n",
                  byte_cmd, nomeDirecao(dir_bits), tempo_duracao_alvo, pwm_val);
  }
}

// ====================================================================
// DECODIFICADOR SDR MANCHESTER (3,5 MHz)
// ====================================================================
void processarPacote(int total_bordas) {
  uint32_t soma_duracao = 0;
  int amostras = 24; 
  if (total_bordas < 50) return; 
  for(int i = 10; i < 10 + amostras; i++) soma_duracao += (tempo_bordas[i] - tempo_bordas[i-1]);
  uint32_t media_meio_bit = soma_duracao / amostras;

  uint8_t* fluxo_bruto = (uint8_t*)malloc(MAX_BORDAS * 2);
  if(!fluxo_bruto) return;
  int half_bits_total = 0;

  for (int i = 1; i < total_bordas; i++) {
     uint32_t duracao = tempo_bordas[i] - tempo_bordas[i-1];
     uint8_t estado = nivel_bordas[i-1]; 
     int proporcao = (duracao + (media_meio_bit / 2)) / media_meio_bit;
     if (proporcao > 2) proporcao = 2; 
     if (proporcao < 1) proporcao = 1;
     for(int k = 0; k < proporcao; k++) {
        if(half_bits_total < (MAX_BORDAS * 2)) fluxo_bruto[half_bits_total++] = estado;
     }
  }

  uint8_t sync_normal[16]    = {1,0, 1,0, 1,0, 1,0, 1,0, 0,1, 1,0, 0,1};
  uint8_t sync_invertido[16] = {0,1, 0,1, 0,1, 0,1, 0,1, 1,0, 0,1, 1,0};
  int payload_start_idx = -1;
  bool sinal_invertido = false;
  
  for (int i = 0; i <= half_bits_total - 16; i++) {
      bool match_normal = true; bool match_invertido = true;
      for (int j = 0; j < 16; j++) {
          if (fluxo_bruto[i+j] != sync_normal[j])    match_normal = false;
          if (fluxo_bruto[i+j] != sync_invertido[j]) match_invertido = false;
      }
      if (match_normal || match_invertido) {
          payload_start_idx = i + 16; 
          sinal_invertido = match_invertido;
          break;
      }
  }
  
  if (payload_start_idx != -1) {
      uint8_t payload[32];
      for(int i = 0; i < 32; i++) { 
          uint8_t byte_val = 0;
          for(int b = 0; b < 8; b++) {
              int idx = payload_start_idx + (i * 16) + (b * 2);
              if (idx + 1 >= half_bits_total) break; 
              uint8_t par = (fluxo_bruto[idx] << 1) | fluxo_bruto[idx+1];
              if (!sinal_invertido && par == 0b10) byte_val |= (1 << (7 - b));
              else if (sinal_invertido && par == 0b01) byte_val |= (1 << (7 - b));
          }
          payload[i] = byte_val;
      }
      processarComandoApp(payload);
  }
  free(fluxo_bruto); 
}

void setup() {
  Serial.begin(115200);
  pinMode(RX_PIN, INPUT_PULLDOWN);

  // Inicializa os pinos de direção da L293DNE
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Inicializa o PWM de Hardware nos pinos Enable (API ESP32 Core 3.x)
  ledcAttach(EN1, PWM_freq, PWM_res);
  ledcAttach(EN2, PWM_freq, PWM_res);
  parar();

  Serial.println("Receptor SDR AGV + Controle L293DNE Iniciado.");
}

void loop() {
  // 1. Mantém a máquina de estados e temporização dos motores rodando sem bloquear
  gerenciarMotores();

  // 2. Escuta o canal óptico VLC
  if (digitalRead(RX_PIN) != 0) {
    portDISABLE_INTERRUPTS(); 
    int total_bordas = 0;
    uint32_t ciclo_atual = ESP.getCycleCount();
    uint32_t ultimo_ciclo = ciclo_atual;
    uint8_t estado_atual = (REG_READ(GPIO_IN_REG) >> RX_PIN) & 0x01; 
    
    tempo_bordas[total_bordas] = ciclo_atual;
    nivel_bordas[total_bordas] = estado_atual;
    total_bordas++;

    const uint32_t TIMEOUT_PACOTE = 1200000; 
    while(total_bordas < MAX_BORDAS) {
      ciclo_atual = ESP.getCycleCount();
      if (ciclo_atual - ultimo_ciclo > TIMEOUT_PACOTE) break; 

      uint8_t novo_estado = (REG_READ(GPIO_IN_REG) >> RX_PIN) & 0x01;
      if (novo_estado != estado_atual) {
        estado_atual = novo_estado;
        tempo_bordas[total_bordas] = ciclo_atual;
        nivel_bordas[total_bordas] = estado_atual;
        ultimo_ciclo = ciclo_atual;
        total_bordas++;
      }
    }
    portENABLE_INTERRUPTS(); 

    if (total_bordas > 100) processarPacote(total_bordas);
  }
}