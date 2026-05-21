#ifndef MQTT_WEBSOCKET_CORRECT_H
#define MQTT_WEBSOCKET_CORRECT_H
 
#include <WiFi.h>   //added
#include <WiFiClientSecure.h>   //added
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include "Configs.h"
#include "Movimento.h"
#include "CoreoBritney.h"
#include "mp3.h"
#include "OtaSetup.h"   //added
 
// ─────────────────────────────────────────────────────────
//  PROTOCOLO MQTT-OVER-WEBSOCKET (porta 8084 / WSS)
//
//  O broker espera frames BINÁRIOS no formato MQTT 3.1.1.
//  Não é texto: cada pacote tem um cabeçalho de controle +
//  tamanho restante + payload, tudo em bytes.
//
//  Fluxo:
//    1. WebSocket conecta em wss://host:8084/mqtt
//    2. Enviamos pacote CONNECT (binário)
//    3. Broker responde CONNACK  → subscrevemos
//    4. Broker manda PUBLISH     → processamos
//    5. Keep-alive via PINGREQ a cada ~30 s
// ─────────────────────────────────────────────────────────
 
extern DFRobotDFPlayerMini player;
extern bool coreografiaEmExecucao;
extern WebSocketsClient webSocket;
 
// ── Variável de estado ────────────────────────────────────
static unsigned long lastPing = 0;
static const unsigned long PING_INTERVAL_MS = 30000;
 
// ════════════════════════════════════════════════════════
//  MONTADORES DE PACOTES MQTT BINÁRIOS
// ════════════════════════════════════════════════════════
 
// Escreve o campo "Remaining Length" do MQTT (codificação variável)
static int writeRemainingLength(uint8_t* buf, int len) {
    int idx = 0;
    do {
        uint8_t encodedByte = len % 128;
        len /= 128;
        if (len > 0) encodedByte |= 0x80;
        buf[idx++] = encodedByte;
    } while (len > 0);
    return idx;
}
 
// Escreve string prefixada por 2 bytes de tamanho (big-endian)
static int writeMQTTString(uint8_t* buf, const char* str) {
    uint16_t len = strlen(str);
    buf[0] = (len >> 8) & 0xFF;
    buf[1] = len & 0xFF;
    memcpy(buf + 2, str, len);
    return 2 + len;
}
 
static void sendMQTTConnect() {
    String clientId = "ESP32_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    const char* cid = clientId.c_str();

    uint8_t payload[256];
    int pi = 0;

    // Variable header
    pi += writeMQTTString(payload + pi, "MQTT"); // protocol name
    payload[pi++] = 0x04;  // protocol level 3.1.1

    // Connect flags
    uint8_t connectFlags = 0x02; // Clean Session
    bool hasUser = (mqtt_user != NULL && strlen(mqtt_user) > 0);
    bool hasPass = (mqtt_password != NULL && strlen(mqtt_password) > 0);
    if (hasUser) connectFlags |= 0x80; // Username flag
    if (hasPass) connectFlags |= 0x40; // Password flag
    payload[pi++] = connectFlags;

    payload[pi++] = 0x00;  // keep-alive MSB (60s)
    payload[pi++] = 0x3C;  // keep-alive LSB

    // Payload: clientId
    pi += writeMQTTString(payload + pi, cid);

    // Payload: username (se existir)
    if (hasUser) {
        pi += writeMQTTString(payload + pi, mqtt_user);
    }

    // Payload: password (se existir)
    if (hasPass) {
        pi += writeMQTTString(payload + pi, mqtt_password);
    }

    // Fixed header
    uint8_t fixedHeader[5];
    fixedHeader[0] = 0x10; // CONNECT
    int rlLen = writeRemainingLength(fixedHeader + 1, pi);

    uint8_t packet[300];
    memcpy(packet, fixedHeader, 1 + rlLen);
    memcpy(packet + 1 + rlLen, payload, pi);

    webSocket.sendBIN(packet, 1 + rlLen + pi);
    Serial.printf("📤 CONNECT enviado (user=%s)\n", hasUser ? mqtt_user : "anonimo");
}
 
// ── SUBSCRIBE ─────────────────────────────────────────────
static void sendMQTTSubscribe(const char* topic, uint16_t packetId) {
    uint8_t payload[128];
    int pi = 0;
 
    // Variable header: Packet Identifier
    payload[pi++] = (packetId >> 8) & 0xFF;
    payload[pi++] = packetId & 0xFF;
 
    // Payload: topic filter + QoS
    pi += writeMQTTString(payload + pi, topic);
    payload[pi++] = 0x00; // QoS 0
 
    uint8_t fixedHeader[5];
    fixedHeader[0] = 0x82; // SUBSCRIBE, reserved bits = 0b0010
    int rlLen = writeRemainingLength(fixedHeader + 1, pi);
 
    uint8_t packet[200];
    memcpy(packet, fixedHeader, 1 + rlLen);
    memcpy(packet + 1 + rlLen, payload, pi);
 
    webSocket.sendBIN(packet, 1 + rlLen + pi);
    Serial.printf("📤 SUBSCRIBE '%s'\n", topic);
}
 
// ── PUBLISH (QoS 0) ───────────────────────────────────────
static void sendMQTTPublish(const char* topic, const char* message) {
    uint8_t payload[256];
    int pi = 0;
 
    pi += writeMQTTString(payload + pi, topic);
    // QoS 0: sem Packet Identifier
    int msgLen = strlen(message);
    memcpy(payload + pi, message, msgLen);
    pi += msgLen;
 
    uint8_t fixedHeader[5];
    fixedHeader[0] = 0x30; // PUBLISH, QoS 0
    int rlLen = writeRemainingLength(fixedHeader + 1, pi);
 
    uint8_t packet[300];
    memcpy(packet, fixedHeader, 1 + rlLen);
    memcpy(packet + 1 + rlLen, payload, pi);
 
    webSocket.sendBIN(packet, 1 + rlLen + pi);
}
 
// ── PINGREQ ───────────────────────────────────────────────
static void sendMQTTPingReq() {
    uint8_t packet[2] = {0xC0, 0x00};
    webSocket.sendBIN(packet, 2);
}
 
// ════════════════════════════════════════════════════════
//  PARSER DE PACOTES MQTT RECEBIDOS
// ════════════════════════════════════════════════════════
 
static void parseMQTTPacket(uint8_t* data, size_t len);
void processMQTTCommand(const String& message);
 
static void parseMQTTPacket(uint8_t* data, size_t len) {
    if (len < 2) return;
 
    uint8_t packetType = (data[0] & 0xF0) >> 4;
 
    switch (packetType) {
        case 2: { // CONNACK
            uint8_t returnCode = data[3];
            if (returnCode == 0x00) {
                Serial.println("✅ MQTT CONNACK: conectado!");
                mqttConnected = true;
                sendMQTTSubscribe("cmd",    1);
                sendMQTTSubscribe("status", 2);
                sendMQTTPublish("status", "ESP32 Online");
            } else {
                Serial.printf("❌ CONNACK erro: 0x%02X\n", returnCode);
            }
            break;
        }
 
        case 3: { // PUBLISH
            // Decodifica Remaining Length
            int pos = 1;
            int remainingLength = 0;
            int multiplier = 1;
            while (pos < (int)len) {
                uint8_t b = data[pos++];
                remainingLength += (b & 0x7F) * multiplier;
                multiplier *= 128;
                if (!(b & 0x80)) break;
            }
 
            // Topic
            if (pos + 2 > (int)len) return;
            uint16_t topicLen = (data[pos] << 8) | data[pos + 1];
            pos += 2;
            if (pos + topicLen > (int)len) return;
            String topic = String((char*)(data + pos), topicLen);
            pos += topicLen;
 
            // QoS 0: sem packet identifier
            uint8_t qos = (data[0] & 0x06) >> 1;
            if (qos > 0) pos += 2; // pula Packet Identifier
 
            // Payload
            int payloadLen = remainingLength - 2 - topicLen - (qos > 0 ? 2 : 0);
            if (payloadLen < 0 || pos + payloadLen > (int)len) return;
            String payload = String((char*)(data + pos), payloadLen);
 
            Serial.printf("📨 PUBLISH  topic='%s'  msg='%s'\n",
                          topic.c_str(), payload.c_str());
 
            if (topic == "cmd") {
                processMQTTCommand(payload);
            }
            break;
        }
 
        case 13: // PINGRESP
            Serial.println("🏓 PINGRESP recebido");
            break;
 
        case 9: // SUBACK
            Serial.println("✅ SUBACK recebido");
            break;
 
        default:
            Serial.printf("📦 Pacote MQTT tipo %d ignorado\n", packetType);
            break;
    }
}
 
// ════════════════════════════════════════════════════════
//  LÓGICA DE COMANDOS
// ════════════════════════════════════════════════════════
 
void processMQTTCommand(const String& message) {
    Serial.printf("🎮 Comando: %s\n", message.c_str());
 
    if (!message.startsWith("DN0")) return;
 
    if (message == "DN0CPA") {
        para();
        coreografiaEmExecucao = false;
        Serial.println("🛑 Parado");
    }
    else if (message == "DN0CG") {
        if (!coreografiaEmExecucao) {
            coreografiaEmExecucao = true;
            coreo();
            coreografiaEmExecucao = false;
        }
    }
    else if (message.startsWith("DN0CL")) {
        int ledNum = message.substring(5).toInt();
        if (ledNum >= 0 && ledNum < 8) {
            digitalWrite(led[ledNum], HIGH);
            Serial.printf("💡 LED %d ligado\n", ledNum);
        }
    }
    else if (message.startsWith("DN0CD")) {
        int ledNum = message.substring(5).toInt();
        if (ledNum >= 0 && ledNum < 8) {
            digitalWrite(led[ledNum], LOW);
            Serial.printf("💡 LED %d desligado\n", ledNum);
        }
    }
    else if (message.startsWith("DN0CM")) {
        int musicaId = message.substring(5).toInt();
        Serial.printf("🎵 Tocar música %d\n", musicaId);
        player.play(musicaId);
    }
    else if (message.startsWith("DN0MF")) { movF(500); }
    else if (message.startsWith("DN0MT")) { movT(500); }
    else if (message.startsWith("DN0GD")) { girD(500); }
    else if (message.startsWith("DN0GE")) { girE(500); }
    else if (message.startsWith("DN0CA")) { calibrarMotores(); }
    else if (message.indexOf('X') != -1 && message.indexOf('Y') != -1) {
        int xPos = message.indexOf('X');
        int yPos = message.indexOf('Y');
        char xDir = message.charAt(xPos + 1);
        char yDir = message.charAt(yPos + 1);
        int velX  = message.charAt(xPos + 2) - '0';
        int velY  = message.charAt(yPos + 2) - '0';
        if (xDir == '-') velX = -velX;
        if (yDir == '-') velY = -velY;
        PWM(velX, velY);
        Serial.printf("🕹️  Movimento: X=%d, Y=%d\n", velX, velY);
    }
}
 
// ════════════════════════════════════════════════════════
//  CALLBACK DO WEBSOCKET
// ════════════════════════════════════════════════════════
 
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("❌ WebSocket desconectado – aguardando reconexão...");
            mqttConnected = false;
            break;
 
        case WStype_CONNECTED:
            Serial.println("🔗 WebSocket conectado – enviando MQTT CONNECT...");
            sendMQTTConnect();
            break;
 
        case WStype_BIN:
            // Pacotes MQTT chegam como frames binários
            parseMQTTPacket(payload, length);
            break;
 
        case WStype_TEXT:
            // Alguns brokers mandam texto em erros; loga e ignora
            Serial.printf("⚠️  Frame TEXT inesperado: %s\n", (char*)payload);
            break;
 
        case WStype_ERROR:
            Serial.println("❌ Erro WebSocket");
            break;
 
        case WStype_PING:
        case WStype_PONG:
            break;
    }
}
 
// ════════════════════════════════════════════════════════
//  INICIALIZAÇÃO
// ════════════════════════════════════════════════════════
 
void initMQTTWebSocket() {
    Serial.printf("🔌 Conectando: wss://%s:%d%s\n",
                  mqtt_server, mqtt_port, mqtt_path);
 
    // Subprotocolo obrigatório para MQTT-over-WebSocket
    webSocket.setExtraHeaders("Sec-WebSocket-Protocol: mqtt");
    webSocket.setReconnectInterval(5000);
 
    webSocket.beginSSL(mqtt_server, mqtt_port, mqtt_path);
    webSocket.onEvent(webSocketEvent);
}
 
// ════════════════════════════════════════════════════════
//  LOOP – chame no loop() principal
// ════════════════════════════════════════════════════════
 
void mqttLoop() {
    webSocket.loop();
 
    // Keep-alive MQTT manual
    if (mqttConnected && (millis() - lastPing > PING_INTERVAL_MS)) {
        lastPing = millis();
        sendMQTTPingReq();
    }
}
 
#endif // MQTT_WEBSOCKET_CORRECT_H
 