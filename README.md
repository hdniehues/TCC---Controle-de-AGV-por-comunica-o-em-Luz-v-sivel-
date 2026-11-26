# TCC---Controle-de-AGV-por-comunica-o-em-Luz-v-sivel-
# Controle de AGV por Comunicação em Luz Visível (VLC)

Repositório oficial com os códigos desenvolvidos no Trabalho de Conclusão de Curso (Engenharia de Controle e Automação — UTFPR, Campus Curitiba, 2026).

**Autor:** Hugo Dias Niehues  
**Orientador:** Prof. Dr. Juan Camilo Castellanos Rodriguez  

## Arquitetura do Sistema
- **Microcontroladores:** 2x ESP32-S3 N16R8 (Emissor e Receptor)
- **Camada Física / Enlace:** Modulação OOK a 3,5 MHz (throughput útil de 1,75 Mbps) com codificação Manchester gerada via periférico SPI (pino MOSI com DMA) e decodificação SDR em *bare-metal* no receptor.
- **Receptor Óptico:** Fotodiodo de avalanche Hamamatsu C12702-12 + isolamento por optoacoplador 6N137.
- **Atuação:** Driver dupla ponte H L293DNE controlando dois motores DC (cinemática diferencial).

## Estrutura do Repositório
- `/emissor_vlc`: Firmware do ESP32-S3 transmissor (Servidor Web embarcado via Wi-Fi AP, montagem dos pacotes de 32 bytes, codificação Manchester e disparo SPI/DMA).
- `/receptor_vlc_agv`: Firmware do ESP32-S3 receptor (captura de bordas via `GPIO_IN_REG` e `ESP.getCycleCount()`, recuperação dinâmica de clock pelo preâmbulo, decodificação Manchester e máquina de estados não-bloqueante para controle PWM dos motores).

## Bibliotecas Necessárias (Arduino IDE)
Para compilar o código do **Emissor**, instale:
- `AsyncTCP`
- `ESPAsyncWebServer`
