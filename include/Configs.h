#ifndef CONFIGS_H
#define CONFIGS_H

#include <WiFi.h>
#include <WiFiClientSecure.h> 
#include <PubSubClient.h>
#include <EEPROM.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

//Versão do Codigo
const char* version = "V3.0-novorobo";

//WIFI
const char* ssid = "baile10";
const char* password = "11223344";
const char* host = "Carrinho";

// ID único do robô - gravado na EEPROM
#define ID_ROBO 0
char id_robo[16] = "robo1";  // Alterar a medida que troca de robo

// Configurações MQTT
WebSocketsClient webSocket;
bool mqttConnected = false;

const char* mqtt_server = "e2792d91.ala.us-east-1.emqxsl.com";
const int mqtt_port = 8084;
const char* mqtt_path = "/mqtt";
const char* mqtt_user = "baile";     
const char* mqtt_password = "baile10";   

// Parâmetros do robô (para odometria e pid)
const float BASE_RODAS = 0.230;      // Distância entre rodas (metros)
const float RAIO_RODAS = 0.033;    // Raio da roda (metros)
const float CIRCUNF_RODAS = 2 * PI * RAIO_RODAS;
const float PULSOS_POR_REV = 20.0;       // Pulsos por revolução do encoder (número de buracos)
const float REDUCAO = 48.0; // Caixa de redução 48:1
const float PULSOS_POR_REV_TOTAL = PULSOS_POR_REV * REDUCAO;
const float DIST_POR_PULSO = CIRCUNF_RODAS / PULSOS_POR_REV;

/* tem caixa de redução 48:1 */

// ===== Compatibilidade de nomes (Português <-> Inglês) =====
// Algumas unidades/headers do projeto usam nomes em inglês (ex: Odometry.h)
// enquanto este arquivo usa nomes em português. Para manter compatibilidade
// definimos aliases via macros para os nomes esperados nas outras unidades.

#define WHEEL_BASE BASE_RODAS
#define DIST_PER_PULSE DIST_POR_PULSO

// IDs / nomes do robô
#define robotId id_robo

// Variáveis de odometria / pose
#define robotX roboX
#define robotY roboY
#define robotTheta roboTheta
#define robotVx robotVxLinear
#define robotVy robotVyLinear
#define robotVtheta roboVthetaAngular

// Encoders / tempos
#define lastPulsosEncoderE ultimoPulsosEncoderE
#define lastPulsosEncoderD ultimoPulsosEncoderD
#define lastOdometryTime ultimoTempoOdometria

// LEDs
#define NUM_LEDS num_leds

// MP3 pins (alguns arquivos usam PIN_MP3_TX/RX)
#define PIN_MP3_TX MP3_TX
#define PIN_MP3_RX MP3_RX

// ============================================================

//PWM
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

// Variáveis para PWM
float fatorCorrecaoEsquerda = 1.0;
float fatorCorrecaoDireita = 1.0;
bool calibracaoAtiva = false;
unsigned long tempoCalibracao = 0;
int pulsosAlvoCalibracao = 100;
//Variáveis para PID
const float Kp = 1.5; //erro atual
const float Ki = 0.05; //erro acumulado
const float Kd = 0.1; //prevê erro
float erroAnterior = 0;
float erroIntegral = 0;
float erroDerivativo = 0;
float distTotalPercorrida = 0.0;
int velocDesejada = 0;
int rotacDesejada = 0;
float velocRealDireita = 0;
float velocRealEsquerda = 0;
float correcaoPID = 0;
float correcaoMaxPID = 3.0;


// PORTAS
//Encoder
const int encoderE = 27;
const int encoderD = 14;
// Motores
const int motor1Pin1 = 25;                      
const int motor1Pin2 = 26;                          
const int motor2Pin1 = 32;                    
const int motor2Pin2 = 33;

//Giroscópio
const int sda_gyro = 21;
const int slc_gyro = 22;
//MP3
static const uint8_t MP3_TX = 12;
static const uint8_t MP3_RX = 13;

const int srclk = 23;
const int rclk = 19;
const int seri = 18;
                     

// LEDs
const int num_leds = 8;
int led[num_leds] = {2, 4, 5, 12, 13, 14, 15, 16}; 
uint8_t estadoLeds = 0x00;
int iled = 0;

bool coreografiaEmExecucao = false;
unsigned long tempo = 0;
unsigned long tempoLimite;
long ultTentativaReconex = 0;

String inputString = "";
bool stringCompleta = false;
bool otaIsOpen = false;

// Variáveis para os encoders
volatile int pulsosEncoderE = 0;
volatile int pulsosEncoderD = 0;
volatile int ultimoPulsosEncoderE = 0;
volatile int ultimoPulsosEncoderD = 0;
unsigned long ultimoTempoOdometria = 0;

// Posição inicial do robô (para odometria)
float roboX = 0.0;
float roboY = 0.0;
float roboTheta = 0.0;
float roboVxLinear = 0.0;
float roboVyLinear = 0.0;
float roboVthetaAngular = 0.0;

// Funções de interrupção para o encoder 
/*IRAM_ATTR: um macro para colocar a função na memória IRAM, para que ela possa 
ser executava repidamente, mesmo quando a memória Flash estiver inacessível.Em 
outras palavras, utilizada em rotinas de interrupção para que não seja  parado 
o programa principal para executar essa interrupção, uma vez que é muito rápido
*/
void IRAM_ATTR interrupcaoEncoderE() {
  pulsosEncoderE++;
}
void IRAM_ATTR interrupcaoEncoderD() {
  pulsosEncoderD++;
}

// Configura os encoders
void SetupEncoders() {
  pinMode(encoderE, INPUT_PULLUP);
  pinMode(encoderD, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(pino), a interrupção, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderE), interrupcaoEncoderE, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderD), interrupcaoEncoderD, RISING);
  Serial.println("✅ Encoders configurados");
}

void resetEncoders() {
  pulsosEncoderE = 0;
  pulsosEncoderD = 0;
  ultimoPulsosEncoderE = 0;
  ultimoPulsosEncoderD = 0;
}

//CONFIGURAÇÕES DA IDENTIFICAÇÃO DO ROBÔ
// Carrega ID 
void carregarIdRobo() {
  EEPROM.begin(32);
  char storedId[16];
  for (int i = 0; i < 15; i++) {
    storedId[i] = EEPROM.read(ID_ROBO + i);
    if (storedId[i] == 0) break;
  }
  storedId[15] = '\0';
  if (strlen(storedId) > 0) {
    strcpy(id_robo, storedId);
  }
  EEPROM.end();
  Serial.printf("🤖 Robô ID: %s\n", id_robo);
}

// Salva ID 
void salvarIdRobo(const char* novoIdRobo) {
  EEPROM.begin(32);
  for (int i = 0; i < 16 && novoIdRobo[i] != '\0'; i++) {
    EEPROM.write(ID_ROBO + i, novoIdRobo[i]);
  }
  EEPROM.commit();
  EEPROM.end();
  strcpy(id_robo, novoIdRobo);
  Serial.printf("💾 ID do robô salvo: %s\n", id_robo);
}

int eepromNumero() {
  EEPROM.begin(12);
  float temp2;
  EEPROM.get(0, temp2);
  int tempint = temp2 + 48;
  EEPROM.end();
  return tempint;
}

void gravarEeprom(int temp) {
  int num = temp;
  EEPROM.begin(12);
  EEPROM.writeFloat(0, num);
  EEPROM.end();
}

#endif
