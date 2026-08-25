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
const char* version = "V2.0-MULTIROBOT";

//WIFI
const char* ssid = "baile10";
const char* password = "11223344";
const char* host = "Carrinho";

// ID único do robô (gravado na EEPROM)
#define ROBOT_ID_ADDR 0
char robotId[16] = "robo1";  // Padrão, pode ser alterado via MQTT

WebSocketsClient webSocket;
bool mqttConnected = false;

// Configurações MQTT
const char* mqtt_server = "e2792d91.ala.us-east-1.emqxsl.com";
const int mqtt_port = 8084;
const char* mqtt_path = "/mqtt";
const char* mqtt_user = "baile";     
const char* mqtt_password = "baile10";   

// Parâmetros do robô (para odometria)
const float WHEEL_BASE = 0.145;      // Distância entre rodas (metros)
const float WHEEL_RADIUS = 0.033;    // Raio da roda (metros)
const int PULSES_PER_REV = 20;       // Pulsos por revolução do encoder
const float WHEEL_CIRC = 2 * PI * WHEEL_RADIUS;
const float DIST_PER_PULSE = WHEEL_CIRC / PULSES_PER_REV;

//PWM
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

//MSG
char mensagem[12];

// Variáveis para correção automática
float fatorCorrecaoEsquerda = 1.0;
float fatorCorrecaoDireita = 1.0;
bool calibracaoAtiva = false;
unsigned long tempoCalibracao = 0;
int pulsosAlvoCalibracao = 100;

// Portas
const int sda_gyro = 21;
const int slc_gyro = 22;

static const uint8_t PIN_MP3_TX = 12;
static const uint8_t PIN_MP3_RX = 13;

const int srclk = 23;
const int rclk = 19;
const int seri = 18;

const int encoderE = 27;
const int encoderD = 14;

// Motores
const int motor1Pin1 = 25;                      
const int motor1Pin2 = 26;                          
const int motor2Pin1 = 32;                    
const int motor2Pin2 = 33;                     

// LEDs
const int NUM_LEDS = 8;
int led[NUM_LEDS] = {2, 4, 5, 12, 13, 14, 15, 16}; 
uint8_t estadoLeds = 0x00;
int iled = 0;

bool coreografiaEmExecucao = false;
unsigned long tempo = 0;
unsigned long timeLimit;
long lastReconnectAttempt = 0;

String inputString = "";
bool stringComplete = false;
bool otaIsOpen = false;

// Variáveis para os encoders
volatile int pulsosEncoderE = 0;
volatile int pulsosEncoderD = 0;
volatile int lastPulsosEncoderE = 0;
volatile int lastPulsosEncoderD = 0;
unsigned long lastOdometryTime = 0;

// Posição do robô (para odometria)
float robotX = 0.0;
float robotY = 0.0;
float robotTheta = 0.0;
float robotVx = 0.0;
float robotVy = 0.0;
float robotVtheta = 0.0;

// Função de interrupção para o encoder esquerdo
void IRAM_ATTR interrupcaoEncoderE() {
  pulsosEncoderE++;
}

// Função de interrupção para o encoder direito
void IRAM_ATTR interrupcaoEncoderD() {
  pulsosEncoderD++;
}

// Configura os encoders
void SetupEncoders() {
  pinMode(encoderE, INPUT_PULLUP);
  pinMode(encoderD, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderE), interrupcaoEncoderE, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderD), interrupcaoEncoderD, RISING);
  Serial.println("✅ Encoders configurados");
}

void resetEncoders() {
  pulsosEncoderE = 0;
  pulsosEncoderD = 0;
  lastPulsosEncoderE = 0;
  lastPulsosEncoderD = 0;
}

// Carrega ID do robô da EEPROM
void loadRobotId() {
  EEPROM.begin(32);
  char storedId[16];
  for (int i = 0; i < 15; i++) {
    storedId[i] = EEPROM.read(ROBOT_ID_ADDR + i);
    if (storedId[i] == 0) break;
  }
  storedId[15] = '\0';
  if (strlen(storedId) > 0) {
    strcpy(robotId, storedId);
  }
  EEPROM.end();
  Serial.printf("🤖 Robô ID: %s\n", robotId);
}

// Salva ID do robô na EEPROM
void saveRobotId(const char* newId) {
  EEPROM.begin(32);
  for (int i = 0; i < 16 && newId[i] != '\0'; i++) {
    EEPROM.write(ROBOT_ID_ADDR + i, newId[i]);
  }
  EEPROM.commit();
  EEPROM.end();
  strcpy(robotId, newId);
  Serial.printf("💾 ID salvo: %s\n", robotId);
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
