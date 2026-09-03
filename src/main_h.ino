#include <Arduino.h>
#include "Biblioteca.h"
#include "Configs.h"
#include "OtaSetup.h"
#include "MQTTWebSocketCorrect.h"

SoftwareSerial softwareSerial(MP3_RX, MP3_TX);
DFRobotDFPlayerMini player;

void setup() {
  Serial.begin(115200);
  delay(100);
  //salvarIdRobo("robo2          ");
  Serial.println("\n=== Carrinho 10 Dimensões ===");
  Serial.print("Versão: ");
  Serial.println(version);

  SetupLeds();
  Mp3Setup();
  SetupEncoders();
  carregarIdRobo();
  localization.setup();
  multiRobotCoord.setup();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  initMQTTWebSocket();
  OTASetup();

  Serial.println("✅ Sistema pronto!!");
  Serial.println("Aguardando comandos MQTT via WSS...");
}

void loop() {
  mqttLoop();
  server.handleClient();
  delay(10);
}
