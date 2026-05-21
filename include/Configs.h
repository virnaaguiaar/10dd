#ifndef CONFIGS_H
#define CONFIGS_H

#include <WiFi.h>
#include <WiFiClientSecure.h> 
#include <PubSubClient.h>
#include <EEPROM.h>

#include <WebSocketsClient.h>  // Instale esta biblioteca
#include <ArduinoJson.h>

#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

//Versão do Codigo
const char* version = "V1.0";
//WIFI
const char* ssid = "sensacaofutebol";
const char* password = "12345678";
const char* host = "Carrinho";


WebSocketsClient webSocket;
bool mqttConnected = false;

// Configurações
const char* mqtt_server = "wf671196.ala.us-east-1.emqxsl.com";
const int mqtt_port = 8084;  // MQTT over WebSocket
const char* mqtt_path = "/mqtt";
const char* mqtt_user = "baile";     // Username
const char* mqtt_password = "baile10";   // Password

//PWM
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

//MSG
char mensagem[12]; // Array de caracteres para armazenar mensagens recebidas (12 bytes)

// Variáveis para correção automática
float fatorCorrecaoEsquerda = 1.0;
float fatorCorrecaoDireita = 1.0;
bool calibracaoAtiva = false;
unsigned long tempoCalibracao = 0;
int pulsosAlvoCalibracao = 100; // Quantos pulsos para calibrar

//modificações
const int sda_gyro = 21;
const int slc_gyro = 22;

static const uint8_t PIN_MP3_TX = 12; // Connects to module's RX 
static const uint8_t PIN_MP3_RX = 13; // Connects to module's TX 

const int srclk = 23;
const int rclk = 19;
const int seri = 18;

const int encoderE = 27;
const int encoderD = 14;

//Portas
// Motor A
const int motor1Pin1 = 25;                      
const int motor1Pin2 = 26;                          
// Motor B 
const int motor2Pin1 = 32;                    
const int motor2Pin2 = 33;                     

// LEDs
const int NUM_LEDS = 8;
int led[NUM_LEDS] = {2, 4, 5, 12, 13, 14, 15, 16}; 

uint8_t estadoLeds = 0x00;

int iled = 0;  // Variável auxiliar para iterar sobre os LEDs

bool coreografiaEmExecucao = false;
unsigned long tempo = 0;   // Variável para armazenar tempo de execução
unsigned long timeLimit;   // Variável para limite de tempo das coreografias
long lastReconnectAttempt = 0;  // Última tentativa de reconexão MQTT
 

String inputString = "";      // String para armazenar dados recebidos pela serial
bool stringComplete = false;  // Flag que indica se uma mensagem serial completa foi recebida

bool otaIsOpen = false;   // Flag que indica se a atualização OTA está ativa

// Variáveis para os encoders
volatile int pulsosEncoderE = 0;
volatile int pulsosEncoderD = 0;

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
  // Configura os pinos dos encoders como entrada com pullup
  pinMode(encoderE, INPUT_PULLUP);
  pinMode(encoderD, INPUT_PULLUP);
  
  // Anexa as interrupções
  attachInterrupt(digitalPinToInterrupt(encoderE), interrupcaoEncoderE, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderD), interrupcaoEncoderD, RISING);
  
  Serial.println("Encoders configurados");
}

// Função para resetar os contadores dos encoders
void resetEncoders() {
  pulsosEncoderE = 0;
  pulsosEncoderD = 0;
}

int eepromNumero(){
  EEPROM.begin(12);
  float temp2;
  EEPROM.get(0, temp2);
  int tempint = temp2 + 48;
  EEPROM.end();
  return tempint;
}

void gravarEeprom(int temp){
  int num = temp;
  
  EEPROM.begin(12);
  EEPROM.writeFloat(0, num);
  float temp2;
  EEPROM.get(0, temp2);
  int tempint = temp2 + 48;
  
  EEPROM.end();
}

#endif // CONFIGS_H