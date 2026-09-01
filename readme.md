# Pinagem Oficial - Robô 10 Dimensões

Este documento mapeia todas as conexões físicas do ESP32 Dev Module utilizadas no firmware do projeto.

## Tabela de GPIOs

| Componente / Módulo | Pinos ESP32 (GPIO) | Variáveis no Código |
| :--- | :--- | :--- |
| **Ponte H - Motor Esquerdo** | `25`, `26` | `motor1Pin1`, `motor1Pin2` |
| **Ponte H - Motor Direito** | `32`, `33` | `motor2Pin1`, `motor2Pin2` |
| **Sensor de Rotação (Encoder Esq.)** | `27` | `encoderE` |
| **Sensor de Rotação (Encoder Dir.)** | `14` | `encoderD` |
| **Giroscópio MPU6050 (SDA)** | `21` | `sda_gyro` |
| **Giroscópio MPU6050 (SCL)** | `22` | `slc_gyro` |
| **Módulo de Áudio DFPlayer (TX)** | `12` | `PIN_MP3_TX` |
| **Módulo de Áudio DFPlayer (RX)** | `13` | `PIN_MP3_RX` |
| **Lógica / Shift Register** | `18`, `19`, `23` | `seri`, `rclk`, `srclk` |
| **LEDs de Efeitos (Coreografia)** | `2`, `4`, `5`, `12`, `13`, `14`, `15`, `16` | Array `led[NUM_LEDS]` |

## Detalhes de Funcionamento
* **Motores:** Utilizam 4 canais PWM de alta frequência (1000 Hz) em resolução de 8 bits para controle direcional preciso.
* **Encoders:** Trabalham via interrupção de hardware (modo `RISING`), lendo 20 pulsos por revolução para a odometria e cálculo de colisão.
* **Comunicação:** O MP3 utiliza a biblioteca `SoftwareSerial` nos pinos 12 e 13 operando a 9600 bps, enquanto o Giroscópio trafega via I2C (Wire) a 400kHz.