#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "driver/spi_master.h"

// --- CONFIGURAÇÕES VLC ---
#define MOSI_PIN 11 
const uint32_t SPI_FREQ = 3500000; 

const int PREAMBLE_SIZE = 16; 
const uint8_t PREAMBULO_LOGICO = 0x00; 
const uint8_t START_WORD = 0xFA;       
const int PAYLOAD_SIZE = 32; 
const int RAW_PACKET_SIZE = PREAMBLE_SIZE + 1 + PAYLOAD_SIZE;
const int MANCHESTER_PACKET_SIZE = RAW_PACKET_SIZE * 2;       

uint8_t tx_buffer[MANCHESTER_PACKET_SIZE];
spi_device_handle_t spi_handle;

// --- DEFINIÇÕES DA ARQUITETURA DE BITS [DD][TT][VVVV] ---
#define DIR_TRAS     0b00
#define DIR_FRENTE   0b01
#define DIR_DIREITA  0b10
#define DIR_ESQUERDA 0b11

#define T_0S 0b00
#define T_2S 0b10
#define T_4S 0b01
#define T_6S 0b11

#define VEL_ZERO 0x0
#define VEL_32   0x1
#define VEL_180  0x2
#define VEL_190  0x3
#define VEL_200  0x4
#define VEL_215  0x5
#define VEL_230  0x6
#define VEL_242  0x7
#define VEL_255  0x8

constexpr uint8_t montarByte(uint8_t dir, uint8_t tempo, uint8_t vel_idx) {
  return ((dir & 0x03) << 6) | ((tempo & 0x03) << 4) | (vel_idx & 0x0F);
}

// --- OS 4 TRAJETOS PRÉ-DETERMINADOS ---
const uint8_t ROTA_1_QUADRADO[] = {
  montarByte(DIR_FRENTE,  T_4S, VEL_200),
  montarByte(DIR_DIREITA, T_2S, VEL_230),
  montarByte(DIR_FRENTE,  T_4S, VEL_200),
  montarByte(DIR_DIREITA, T_2S, VEL_230),
  montarByte(DIR_FRENTE,  T_4S, VEL_200),
  montarByte(DIR_DIREITA, T_2S, VEL_230),
  montarByte(DIR_FRENTE,  T_4S, VEL_200),
  montarByte(DIR_DIREITA, T_2S, VEL_230),
  0x00
};

const uint8_t ROTA_2_ESCALONADA[] = {
  montarByte(DIR_FRENTE,   T_4S, VEL_200),
  montarByte(DIR_ESQUERDA, T_2S, VEL_230),
  montarByte(DIR_FRENTE,   T_2S, VEL_200),
  montarByte(DIR_DIREITA,  T_2S, VEL_230),
  montarByte(DIR_FRENTE,   T_4S, VEL_200),
  montarByte(DIR_DIREITA,  T_2S, VEL_230),
  montarByte(DIR_FRENTE,   T_4S, VEL_200),
  montarByte(DIR_DIREITA,  T_2S, VEL_230),
  montarByte(DIR_FRENTE,   T_4S, VEL_200),
  montarByte(DIR_DIREITA,  T_2S, VEL_230),
  montarByte(DIR_FRENTE,   T_2S, VEL_200),
  montarByte(DIR_ESQUERDA, T_2S, VEL_230),
  montarByte(DIR_FRENTE,   T_4S, VEL_200),
  montarByte(DIR_DIREITA,  T_2S, VEL_230),
  0x00
};

const uint8_t ROTA_3_TRIANGULO[] = {
  montarByte(DIR_FRENTE,  T_4S, VEL_200),
  montarByte(DIR_DIREITA, T_2S, VEL_230),
  montarByte(DIR_FRENTE,  T_4S, VEL_200),
  montarByte(DIR_DIREITA, T_4S, VEL_230),
  montarByte(DIR_FRENTE,  T_6S, VEL_215),
  montarByte(DIR_DIREITA, T_2S, VEL_230),
  0x00
};

const uint8_t ROTA_4_TRAPEZIO[] = {
  montarByte(DIR_FRENTE,   T_6S, VEL_200),
  montarByte(DIR_ESQUERDA, T_2S, VEL_230),
  montarByte(DIR_FRENTE,   T_4S, VEL_200),
  montarByte(DIR_ESQUERDA, T_2S, VEL_230),
  montarByte(DIR_FRENTE,   T_2S, VEL_200),
  montarByte(DIR_ESQUERDA, T_2S, VEL_230),
  montarByte(DIR_FRENTE,   T_4S, VEL_200),
  montarByte(DIR_ESQUERDA, T_2S, VEL_230),
  0x00
};

// --- SERVIDOR WEB ---
AsyncWebServer server(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Supervisorio AGV - VLC</title>
  <style>
    :root {
      --bg-dark: #0f172a;
      --card-bg: #1e293b;
      --card-border: #334155;
      --text-main: #f8fafc;
      --text-muted: #94a3b8;
      --accent-blue: #3b82f6;
      --accent-green: #10b981;
      --accent-red: #ef4444;
      --accent-amber: #f59e0b;
    }
    * { box-sizing: border-box; font-family: 'Segoe UI', Roboto, Arial, sans-serif; user-select: none; }
    body { margin: 0; padding: 24px; background: radial-gradient(circle at top, #1e293b 0%, #0f172a 100%); color: var(--text-main); min-height: 100vh; }

    .header-title { text-align: center; margin: 0 0 20px 0; font-size: 22px; font-weight: 700; letter-spacing: 1px; color: #e2e8f0; text-transform: uppercase; }
    .header-title span { color: var(--accent-blue); }

    /* Barra Superior: mantem as posicoes originais */
    .top-bar { display: grid; grid-template-columns: 1fr auto 1fr; align-items: start; gap: 20px; margin-bottom: 28px; }
    @media (max-width: 900px) { .top-bar { grid-template-columns: 1fr; justify-items: center; } .spacer-left { display: none; } }

    .center-controls { display: flex; flex-direction: column; align-items: center; gap: 12px; }

    .btn { padding: 13px 28px; font-size: 14px; font-weight: 700; letter-spacing: 0.6px; cursor: pointer; border: 1px solid rgba(255,255,255,0.12); border-radius: 10px; color: #fff; transition: all 0.2s ease; min-width: 260px; box-shadow: 0 4px 12px rgba(0,0,0,0.3); }
    .btn:hover { filter: brightness(1.1); transform: translateY(-1px); }
    .btn:active { transform: translateY(2px); }

    .btn-emerg { background: linear-gradient(135deg, #ef4444, #b91c1c); box-shadow: 0 4px 15px rgba(239,68,68,0.4); }
    .btn-retomar { background: linear-gradient(135deg, #f59e0b, #d97706); box-shadow: 0 4px 15px rgba(245,158,11,0.35); }

    .mode-group { display: flex; gap: 10px; flex-wrap: wrap; justify-content: center; margin-top: 6px; background: #0f172a; padding: 6px; border-radius: 12px; border: 1px solid var(--card-border); }
    .btn-mode { min-width: 145px; padding: 10px 16px; font-size: 12px; background: transparent; color: var(--text-muted); border: none; box-shadow: none; border-radius: 8px; }
    .btn-mode.active { background: var(--accent-blue); color: #fff; box-shadow: 0 2px 10px rgba(59,130,246,0.45); }

    /* Caixa de Sequencia no canto superior direito */
    .config-box { justify-self: end; width: 100%; max-width: 370px; background: var(--card-bg); border: 1px solid var(--card-border); border-radius: 14px; padding: 16px; box-shadow: 0 8px 20px rgba(0,0,0,0.35); transition: 0.25s; }
    .config-box label { display: block; font-size: 12px; font-weight: 700; color: var(--text-muted); margin-bottom: 8px; letter-spacing: 0.5px; }
    .config-row { display: flex; gap: 8px; }
    .config-row input { flex: 1; padding: 10px 12px; font-size: 15px; font-weight: 600; background: #0f172a; color: #fff; border: 1px solid var(--card-border); border-radius: 8px; text-transform: uppercase; outline: none; letter-spacing: 1px; }
    .config-row input:focus { border-color: var(--accent-blue); }
    .config-row button { padding: 10px 18px; font-weight: 700; font-size: 13px; background: linear-gradient(135deg, #10b981, #059669); color: #fff; border: none; border-radius: 8px; cursor: pointer; transition: 0.2s; }
    .config-row button:hover { filter: brightness(1.1); }
    .preview-chips { display: flex; flex-wrap: wrap; gap: 5px; margin-top: 10px; min-height: 24px; }
    .chip { font-size: 11px; font-weight: 700; padding: 3px 8px; border-radius: 6px; background: rgba(59,130,246,0.2); color: #93c5fd; border: 1px solid rgba(59,130,246,0.4); }

    /* Grid Inferior: Esquerda (Setas) e Direita (4 Trajetos) */
    .main-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 28px; }
    @media (max-width: 850px) { .main-grid { grid-template-columns: 1fr; } }

    .panel-box { background: var(--card-bg); border: 1px solid var(--card-border); border-radius: 16px; padding: 24px; display: flex; flex-direction: column; align-items: center; justify-content: center; box-shadow: 0 10px 25px rgba(0,0,0,0.35); transition: 0.25s; }
    .panel-title { font-size: 13px; font-weight: 700; letter-spacing: 0.8px; margin-bottom: 18px; color: var(--text-muted); text-transform: uppercase; }

    .disabled-overlay { opacity: 0.28; pointer-events: none; filter: grayscale(0.6); }

    /* D-Pad Setas */
    .dpad { display: grid; grid-template-columns: 84px 84px 84px; grid-template-rows: 84px 84px 84px; gap: 12px; align-items: center; justify-items: center; }
    .arrow-btn { width: 80px; height: 80px; background: #0f172a; border: 2px solid var(--card-border); cursor: pointer; display: flex; align-items: center; justify-content: center; border-radius: 16px; transition: all 0.15s; box-shadow: inset 0 2px 4px rgba(255,255,255,0.05), 0 6px 14px rgba(0,0,0,0.4); }
    .arrow-btn:hover { border-color: var(--accent-blue); }
    .arrow-btn:active, .arrow-btn.pressing { background: var(--accent-blue); border-color: #93c5fd; transform: scale(0.95); box-shadow: 0 0 20px rgba(59,130,246,0.6); }
    .arrow-btn svg { width: 42px; height: 42px; stroke: #e2e8f0; stroke-width: 2.5; fill: none; stroke-linejoin: round; }

    /* Grid dos 4 Trajetos */
    .routes-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 18px; width: 100%; max-width: 440px; }
    .route-card { background: #0f172a; border: 1px solid var(--card-border); border-radius: 12px; padding: 16px; cursor: pointer; display: flex; flex-direction: column; align-items: center; transition: all 0.2s; }
    .route-card:hover { border-color: var(--accent-green); transform: translateY(-3px); box-shadow: 0 6px 18px rgba(16,185,129,0.2); }
    .route-card:active { transform: translateY(1px); background: rgba(16,185,129,0.15); }
    .route-card svg { width: 120px; height: 85px; stroke: #38bdf8; stroke-width: 2.5; fill: rgba(56,189,248,0.06); }
    .route-card span { margin-top: 10px; font-size: 12px; font-weight: 700; color: #e2e8f0; letter-spacing: 0.5px; }

    .status-bar { margin-top: 24px; background: #0f172a; border: 1px solid var(--card-border); border-radius: 10px; padding: 12px 18px; text-align: center; font-size: 14px; font-weight: 600; color: #38bdf8; box-shadow: inset 0 2px 6px rgba(0,0,0,0.4); }
  </style>
</head>
<body>

  <h1 class="header-title">Painel de Controle <span>AGV VLC</h1>

  <div class="top-bar">
    <div class="spacer-left"></div>

    <div class="center-controls">
      <button class="btn btn-emerg" onclick="enviarSimples('/emergencia', 'PARADA DE EMERGÊNCIA ENVIADA!')">PARADA DE EMERGÊNCIA</button>
      <button class="btn btn-retomar" onclick="enviarSimples('/liberar', 'RETOMAR MOVIMENTO ENVIADO!')">RETOMAR MOVIMENTO</button>
      
      <div class="mode-group">
        <button id="btnAuto" class="btn btn-mode active" onclick="setModo('AUTO')">MODO AUTOMÁTICO</button>
        <button id="btnManTxt" class="btn btn-mode" onclick="setModo('MAN_TXT')">MANUAL (SEQUÊNCIA)</button>
        <button id="btnManSetas" class="btn btn-mode" onclick="setModo('MAN_SETAS')">MANUAL (SETAS)</button>
      </div>
    </div>

    <div id="boxTexto" class="config-box disabled-overlay">
      <label>SEQUÊNCIA DE TRAJETO (F/D/E + 2/4/6):</label>
      <div class="config-row">
        <input type="text" id="inputTrajeto" placeholder="Ex: F6E4D2F4" oninput="atualizarPreview()">
        <button onclick="enviarDigitado()">ENVIAR</button>
      </div>
      <div id="previewChips" class="preview-chips"></div>
    </div>
  </div>

  <div class="main-grid">
    <!-- CAIXA ESQUERDA: SETAS MANUAIS -->
    <div id="boxSetas" class="panel-box disabled-overlay">
      <div class="panel-title">Controle Manual Contínuo (PWM: 200 | Ciclo: 1,5s)</div>
      <div class="dpad">
        <div></div>
        <button class="arrow-btn"
          onmousedown="iniciarSeta('F', this)" onmouseup="pararSeta(this)" onmouseleave="pararSeta(this)"
          ontouchstart="iniciarSeta('F', this); event.preventDefault();" ontouchend="pararSeta(this)">
          <svg viewBox="0 0 50 50"><polygon points="25,6 44,25 32,25 32,44 18,44 18,25 6,25"/></svg>
        </button>
        <div></div>

        <button class="arrow-btn"
          onmousedown="iniciarSeta('E', this)" onmouseup="pararSeta(this)" onmouseleave="pararSeta(this)"
          ontouchstart="iniciarSeta('E', this); event.preventDefault();" ontouchend="pararSeta(this)">
          <svg viewBox="0 0 50 50"><polygon points="6,25 25,6 25,18 44,18 44,32 25,32 25,44"/></svg>
        </button>
        <div></div>
        <button class="arrow-btn"
          onmousedown="iniciarSeta('D', this)" onmouseup="pararSeta(this)" onmouseleave="pararSeta(this)"
          ontouchstart="iniciarSeta('D', this); event.preventDefault();" ontouchend="pararSeta(this)">
          <svg viewBox="0 0 50 50"><polygon points="44,25 25,6 25,18 6,18 6,32 25,32 25,44"/></svg>
        </button>

        <div></div>
        <button class="arrow-btn"
          onmousedown="iniciarSeta('T', this)" onmouseup="pararSeta(this)" onmouseleave="pararSeta(this)"
          ontouchstart="iniciarSeta('T', this); event.preventDefault();" ontouchend="pararSeta(this)">
          <svg viewBox="0 0 50 50"><polygon points="25,44 44,25 32,25 32,6 18,6 18,25 6,25"/></svg>
        </button>
        <div></div>
      </div>
    </div>

    <!-- CAIXA DIREITA: 4 TRAJETOS AUTOMÁTICOS -->
    <div id="boxRotas" class="panel-box">
      <div class="panel-title">Trajetos Pré-Determinados (Modo Automático)</div>
      <div class="routes-grid">
        <div class="route-card" onclick="enviarSimples('/rota?id=1', 'Rota 1 (Quadrado) enviada para a fila!')">
          <svg viewBox="0 0 120 90"><rect x="25" y="10" width="70" height="70"/></svg>
          <span>1. QUADRADO</span>
        </div>
        <div class="route-card" onclick="enviarSimples('/rota?id=2', 'Rota 2 (Escalonada) enviada para a fila!')">
          <svg viewBox="0 0 120 90"><polygon points="15,22 72,22 72,10 105,10 105,80 72,80 72,68 15,68"/></svg>
          <span>2. ESCALONADA</span>
        </div>
        <div class="route-card" onclick="enviarSimples('/rota?id=3', 'Rota 3 (Triângulo) enviada para a fila!')">
          <svg viewBox="0 0 120 90"><polygon points="22,10 22,80 102,80"/></svg>
          <span>3. TRIÂNGULO</span>
        </div>
        <div class="route-card" onclick="enviarSimples('/rota?id=4', 'Rota 4 (Trapézio) enviada para a fila!')">
          <svg viewBox="0 0 120 90"><polygon points="35,18 85,18 106,80 14,80"/></svg>
          <span>4. TRAPÉZIO</span>
        </div>
      </div>
    </div>
  </div>

  <div id="statusLog" class="status-bar">Modo Atual: AUTOMÁTICO — Selecione um dos 4 trajetos.</div>

  <script>
    let timerSeta = null;
    let setaAtiva = false;

    function setLog(msg) {
      document.getElementById('statusLog').innerText = msg;
    }

    function setModo(modo) {
      pararSeta(null);
      document.getElementById('btnAuto').classList.toggle('active', modo === 'AUTO');
      document.getElementById('btnManTxt').classList.toggle('active', modo === 'MAN_TXT');
      document.getElementById('btnManSetas').classList.toggle('active', modo === 'MAN_SETAS');

      document.getElementById('boxRotas').classList.toggle('disabled-overlay', modo !== 'AUTO');
      document.getElementById('boxTexto').classList.toggle('disabled-overlay', modo !== 'MAN_TXT');
      document.getElementById('boxSetas').classList.toggle('disabled-overlay', modo !== 'MAN_SETAS');

      if (modo === 'AUTO') setLog('Modo AUTOMÁTICO ativado. Escolha um dos 4 trajetos.');
      if (modo === 'MAN_TXT') setLog('Modo SEQUÊNCIA ativado. Digite tudo junto (ex: F6E4D2F4) e clique em Enviar.');
      if (modo === 'MAN_SETAS') setLog('Modo SETAS ativado. Mantenha uma seta pressionada para mover.');
    }

    // Mostra os passos identificados conforme o usuario digita F6E4D2F4
    function atualizarPreview() {
      const raw = document.getElementById('inputTrajeto').value.toUpperCase();
      const container = document.getElementById('previewChips');
      container.innerHTML = '';
      const regex = /([FDET])\s*\+?\s*([246])/g;
      let m;
      let idx = 1;
      while ((m = regex.exec(raw)) !== null) {
        const chip = document.createElement('span');
        chip.className = 'chip';
        chip.innerText = idx + '. ' + m[1] + m[2] + 's';
        container.appendChild(chip);
        idx++;
      }
    }

    function enviarSimples(url, msg) {
      fetch(url).then(r => r.text()).then(t => setLog(msg + ' [' + t + ']'));
    }

    function enviarDigitado() {
      const val = document.getElementById('inputTrajeto').value.trim();
      if (!val) return;
      fetch('/manual_cmd?str=' + encodeURIComponent(val))
        .then(r => r.text())
        .then(t => setLog('Sequência (' + val.toUpperCase() + '): ' + t));
    }

    function iniciarSeta(dir, btnEl) {
      if (setaAtiva) return;
      setaAtiva = true;
      if (btnEl) btnEl.classList.add('pressing');

      const disparar = () => {
        fetch('/seta?dir=' + dir);
        setLog('Seta [' + dir + '] pressionada -> Pacote VLC enviado (PWM: 200)');
      };

      disparar();
      timerSeta = setInterval(disparar, 1500);
    }

    function pararSeta(btnEl) {
      if (btnEl) btnEl.classList.remove('pressing');
      if (!setaAtiva) return;
      setaAtiva = false;
      if (timerSeta) {
        clearInterval(timerSeta);
        timerSeta = null;
      }
      fetch('/seta?dir=P');
      setLog('Seta solta -> Parada imediata enviada.');
    }
  </script>
</body>
</html>)rawliteral";

// --- CODIFICAÇÃO MANCHESTER E DISPARO SPI ---
uint16_t encodeManchester(uint8_t data) {
  uint16_t encoded = 0;
  for (int i = 0; i < 8; i++) {
    bool bit = (data >> (7 - i)) & 0x01;
    if (bit) encoded |= (0b10 << ((7 - i) * 2));
    else     encoded |= (0b01 << ((7 - i) * 2));
  }
  return encoded;
}

void dispararVLC(uint8_t* payload) {
  uint8_t raw_packet[RAW_PACKET_SIZE];
  
  for(int i = 0; i < PREAMBLE_SIZE; i++) raw_packet[i] = PREAMBULO_LOGICO;
  raw_packet[PREAMBLE_SIZE] = START_WORD;
  memcpy(&raw_packet[PREAMBLE_SIZE + 1], payload, PAYLOAD_SIZE);

  int buffer_index = 0;
  for (int i = 0; i < RAW_PACKET_SIZE; i++) {
    uint16_t man = encodeManchester(raw_packet[i]);
    tx_buffer[buffer_index++] = (man >> 8) & 0xFF; 
    tx_buffer[buffer_index++] = man & 0xFF;        
  }

  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = MANCHESTER_PACKET_SIZE * 8; 
  t.tx_buffer = tx_buffer;

  spi_device_polling_transmit(spi_handle, &t);
}

// --- PARSER DE SEQUÊNCIA CONTÍNUA (Ex: "F6E4D2F4" ou "F6 E4 D2 F4") ---
int converterTextoParaBytes(const String& str, uint8_t* destino, int max_passos) {
  int count = 0;
  int i = 0;
  int len = str.length();

  while (i < len && count < max_passos) {
    char c = toupper(str[i]);
    // Ignora espaços ou separadores opcionais
    if (c == ' ' || c == ',' || c == '+' || c == '-' || c == ';') {
      i++;
      continue;
    }

    uint8_t bits_dir = 0;
    if (c == 'F')      bits_dir = DIR_FRENTE;
    else if (c == 'D') bits_dir = DIR_DIREITA;
    else if (c == 'E') bits_dir = DIR_ESQUERDA;
    else if (c == 'T') bits_dir = DIR_TRAS;
    else { i++; continue; }

    i++;
    while (i < len && (str[i] == ' ' || str[i] == '+')) i++;

    if (i >= len) break;
    char t = str[i];
    uint8_t bits_tempo = T_0S;
    if (t == '2')      bits_tempo = T_2S;
    else if (t == '4') bits_tempo = T_4S;
    else if (t == '6') bits_tempo = T_6S;
    else { i++; continue; }

    destino[count++] = montarByte(bits_dir, bits_tempo, VEL_200);
    i++;
  }
  return count;
}

void setup() {
  Serial.begin(115200);

  // 1. Setup SPI VLC
  spi_bus_config_t buscfg = { .mosi_io_num = MOSI_PIN, .miso_io_num = -1, .sclk_io_num = -1, .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 4096 };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
  spi_device_interface_config_t devcfg = { .command_bits = 0, .address_bits = 0, .dummy_bits = 0, .mode = 0, .clock_speed_hz = (int)SPI_FREQ, .spics_io_num = -1, .queue_size = 1 };
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle));

  // 2. Setup Wi-Fi AP
  WiFi.softAP("AGV_Controle", "12345678");
  Serial.print("Servidor Web Iniciado. IP: ");
  Serial.println(WiFi.softAPIP());

  // 3. Rotas Web
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/rota", HTTP_GET, [](AsyncWebServerRequest *request){
    uint8_t payload[PAYLOAD_SIZE] = {0};
    if(request->hasParam("id")) {
      String id = request->getParam("id")->value();
      payload[0] = 0x01;
      
      if(id == "1")      memcpy(&payload[1], ROTA_1_QUADRADO,   sizeof(ROTA_1_QUADRADO));
      else if(id == "2") memcpy(&payload[1], ROTA_2_ESCALONADA, sizeof(ROTA_2_ESCALONADA));
      else if(id == "3") memcpy(&payload[1], ROTA_3_TRIANGULO,  sizeof(ROTA_3_TRIANGULO));
      else if(id == "4") memcpy(&payload[1], ROTA_4_TRAPEZIO,   sizeof(ROTA_4_TRAPEZIO));
      
      dispararVLC(payload);
      Serial.printf("[AUTO] Rota %s disparada via VLC.\n", id.c_str());
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Erro");
    }
  });

  // Recebe sequências completas juntas (ex: F6E4D2F4) até 30 passos num único pacote
  server.on("/manual_cmd", HTTP_GET, [](AsyncWebServerRequest *request){
    uint8_t payload[PAYLOAD_SIZE] = {0};
    if(request->hasParam("str")) {
      String cmdStr = request->getParam("str")->value();
      payload[0] = 0x01; 
      int qtd = converterTextoParaBytes(cmdStr, &payload[1], 30);
      
      if (qtd > 0) {
        dispararVLC(payload);
        Serial.printf("[SEQUENCIA MANUAL] '%s' -> %d movimento(s) empacotado(s): ", cmdStr.c_str(), qtd);
        for (int k = 1; k <= qtd; k++) Serial.printf("0x%02X ", payload[k]);
        Serial.println();
        request->send(200, "text/plain", String(qtd) + " etapa(s) enviada(s) no pacote!");
      } else {
        request->send(400, "text/plain", "Use combinacoes como F6E4D2F4");
      }
    } else {
      request->send(400, "text/plain", "Sem parametro");
    }
  });

  server.on("/seta", HTTP_GET, [](AsyncWebServerRequest *request){
    uint8_t payload[PAYLOAD_SIZE] = {0};
    if(request->hasParam("dir")) {
      String dir = request->getParam("dir")->value();
      payload[0] = 0x02;

      if (dir == "P") {
        payload[1] = 0x00;
      } else {
        uint8_t bits_dir = DIR_FRENTE;
        if (dir == "F")      bits_dir = DIR_FRENTE;
        else if (dir == "D") bits_dir = DIR_DIREITA;
        else if (dir == "E") bits_dir = DIR_ESQUERDA;
        else if (dir == "T") bits_dir = DIR_TRAS;

        payload[1] = montarByte(bits_dir, T_0S, VEL_200);
      }

      dispararVLC(payload);
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Erro");
    }
  });

  server.on("/emergencia", HTTP_GET, [](AsyncWebServerRequest *request){
    uint8_t payload[PAYLOAD_SIZE] = {0};
    payload[0] = 0xEE;
    dispararVLC(payload);
    request->send(200, "text/plain", "0xEE");
  });

  server.on("/liberar", HTTP_GET, [](AsyncWebServerRequest *request){
    uint8_t payload[PAYLOAD_SIZE] = {0};
    payload[0] = 0xDD;
    dispararVLC(payload);
    request->send(200, "text/plain", "0xDD");
  });

  server.begin();
}

void loop() {
}