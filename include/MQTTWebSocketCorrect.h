#ifndef MQTT_WEBSOCKET_CORRECT_H
#define MQTT_WEBSOCKET_CORRECT_H

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include "Configs.h"
#include "Movimento.h"
#include "CoreoBritney.h"
#include "mp3.h"
#include "Odometry.h"
#include "Localization.h"
#include "CollisionAvoidance.h"
#include "MultiRobotCoord.h"

extern DFRobotDFPlayerMini player;
extern bool coreografiaEmExecucao;
extern WebSocketsClient webSocket;
extern char robotId[];
extern float robotX, robotY, robotTheta;

// Variáveis estáticas adicionais
static unsigned long lastPing = 0;
static unsigned long lastPosePublish = 0;
static unsigned long lastStatusPublish = 0;
static unsigned long lastAvoidanceCheck = 0;
static const unsigned long PING_INTERVAL_MS = 30000;
static const unsigned long POSE_PUBLISH_INTERVAL_MS = 100;      // 10Hz
static const unsigned long STATUS_PUBLISH_INTERVAL_MS = 5000;   // 5 segundos
static const unsigned long AVOIDANCE_CHECK_INTERVAL_MS = 50;    // 20Hz

// Flag para controle de movimento autônomo
static bool autoModeEnabled = false;
static int autoTargetX = 0, autoTargetY = 0;

// Forward declarations
void coreo();

// Função para publicar MQTT
void publishMQTT(const char* topic, const char* payload) {
    if (!mqttConnected) return;
    
    // Constrói pacote MQTT publish
    uint8_t packet[1024];  // Aumentado para comportar JSON maior
    int pos = 0;
    
    // Fixed header (PUBLISH)
    packet[pos++] = 0x30; // PUBLISH QoS 0
    
    // Calcula tamanho do payload
    int topicLen = strlen(topic);
    int payloadLen = strlen(payload);
    int remainingLen = 2 + topicLen + payloadLen;
    
    // Encode remaining length
    int rlPos = pos;
    pos++;
    if (remainingLen < 128) {
        packet[rlPos] = remainingLen;
    } else {
        packet[rlPos] = (remainingLen % 128) | 0x80;
        packet[pos++] = remainingLen / 128;
    }
    
    // Topic length
    packet[pos++] = (topicLen >> 8) & 0xFF;
    packet[pos++] = topicLen & 0xFF;
    
    // Topic
    memcpy(packet + pos, topic, topicLen);
    pos += topicLen;
    
    // Payload
    memcpy(packet + pos, payload, payloadLen);
    pos += payloadLen;
    
    webSocket.sendBIN(packet, pos);
}

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

static int writeMQTTString(uint8_t* buf, const char* str) {
    uint16_t len = strlen(str);
    buf[0] = (len >> 8) & 0xFF;
    buf[1] = len & 0xFF;
    memcpy(buf + 2, str, len);
    return 2 + len;
}

static void sendMQTTConnect() {
    String clientId = String(robotId) + "_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    const char* cid = clientId.c_str();

    Serial.printf("📤 MQTT CONNECT: cliente=%s, broker=%s:%d\n", cid, mqtt_server, mqtt_port);

    uint8_t payload[256];
    int pi = 0;

    pi += writeMQTTString(payload + pi, "MQTT");
    payload[pi++] = 0x04;
    uint8_t connectFlags = 0x02;
    bool hasUser = (mqtt_user != NULL && strlen(mqtt_user) > 0);
    bool hasPass = (mqtt_password != NULL && strlen(mqtt_password) > 0);
    if (hasUser) connectFlags |= 0x80;
    if (hasPass) connectFlags |= 0x40;
    payload[pi++] = connectFlags;
    payload[pi++] = 0x00;
    payload[pi++] = 0x3C;
    pi += writeMQTTString(payload + pi, cid);
    if (hasUser) pi += writeMQTTString(payload + pi, mqtt_user);
    if (hasPass) pi += writeMQTTString(payload + pi, mqtt_password);

    uint8_t fixedHeader[5];
    fixedHeader[0] = 0x10;
    int rlLen = writeRemainingLength(fixedHeader + 1, pi);

    uint8_t packet[300];
    memcpy(packet, fixedHeader, 1 + rlLen);
    memcpy(packet + 1 + rlLen, payload, pi);

    webSocket.sendBIN(packet, 1 + rlLen + pi);
}

static void sendMQTTSubscribe(const char* topic, uint16_t packetId) {
    uint8_t payload[128];
    int pi = 0;
    payload[pi++] = (packetId >> 8) & 0xFF;
    payload[pi++] = packetId & 0xFF;
    pi += writeMQTTString(payload + pi, topic);
    payload[pi++] = 0x00;

    uint8_t fixedHeader[5];
    fixedHeader[0] = 0x82;
    int rlLen = writeRemainingLength(fixedHeader + 1, pi);

    uint8_t packet[200];
    memcpy(packet, fixedHeader, 1 + rlLen);
    memcpy(packet + 1 + rlLen, payload, pi);
    webSocket.sendBIN(packet, 1 + rlLen + pi);
    Serial.printf("📥 Inscrevendo no tópico MQTT: %s\n", topic);
}

static void sendMQTTPingReq() {
    uint8_t packet[2] = {0xC0, 0x00};
    webSocket.sendBIN(packet, 2);
}

// Processa comandos recebidos via MQTT
void processMQTTCommand(const String& message) {
    Serial.printf("🎮 Comando: %s\n", message.c_str());

    if (!message.startsWith("DN0")) {
        Serial.println("⚠️ Comando inválido");
        return;
    }

    // PARAR
    if (message == "DN0CPA") {
        Serial.println("🛑 PARAR");
        autoModeEnabled = false;
        pararCoreografia();
        PWM_PID(0, 0);
        desligaLeds();
        return;
    }
    
    // COREOGRAFIA
    else if (message == "DN0CG") {
        Serial.println("🎬 INICIAR COREOGRAFIA");
        iniciarCoreografia();
        return;
    }
    
    // LEDS
    else if (message.startsWith("DN0CL")) {
        int ledNum = message.substring(5).toInt();
        if (ledNum >= 0 && ledNum < 8) {
            digitalWrite(led[ledNum], HIGH);
            Serial.printf("💡 LED %d ON\n", ledNum);
        }
        return;
    }
    else if (message.startsWith("DN0CD")) {
        int ledNum = message.substring(5).toInt();
        if (ledNum >= 0 && ledNum < 8) {
            digitalWrite(led[ledNum], LOW);
            Serial.printf("💡 LED %d OFF\n", ledNum);
        }
        return;
    }
    
    // MÚSICAS
    else if (message.startsWith("DN0CM")) {
        int musicaId = message.substring(5).toInt();
        tocarMusica(musicaId);
        return;
    }
    else if (message == "DN0CPS") {
        pararMusica();
        return;
    }
    
    // MOVIMENTO
    else if (message.indexOf('X') != -1 && message.indexOf('Y') != -1) {
        if (coreografiaEmExecucao) {
            pararCoreografia();
        }
        
        autoModeEnabled = false;
        
        int xPos = message.indexOf('X');
        int yPos = message.indexOf('Y');
        char xDir = message.charAt(xPos + 1);
        char yDir = message.charAt(yPos + 1);
        int velX = message.charAt(xPos + 2) - '0';
        int velY = message.charAt(yPos + 2) - '0';
        if (xDir == '-') velX = -velX;
        if (yDir == '-') velY = -velY;
        
        PWM_PID(velX, velY);
        Serial.printf("🕹️ Movimento: X=%d, Y=%d\n", velX, velY);
        return;
    }
    
    // SET ROBOT ID
    else if (message.startsWith("DN0ID")) {
        String newId = message.substring(5);
        salvarIdRobo(newId.c_str());
        publishMQTT("robot/status", "ID updated");
        return;
    }
    
    // RESET ODOMETRY
    else if (message == "DN0RST") {
        resetarOdometria();
        localization.setup();  // Reinicia localização
        publishMQTT("robot/status", "Odometry reset");
        return;
    }
    
    // MODO DE EVITAÇÃO
    else if (message.startsWith("DN0AVD")) {
        int mode = message.substring(6).toInt();
        collisionAvoidance.setMode((AvoidanceMode)mode);
        char buf[50];
        sprintf(buf, "{\"mode\":%d}", mode);
        publishMQTT("robot/avoidance/mode", buf);
        return;
    }
    
    // RAIO DE SEGURANÇA
    else if (message.startsWith("DN0RAD")) {
        float radius = message.substring(6).toFloat();
        collisionAvoidance.setSafetyRadius(radius);
        char buf[50];
        sprintf(buf, "{\"radius\":%.2f}", radius);
        publishMQTT("robot/avoidance/radius", buf);
        return;
    }
    
    // MODO DE COORDENAÇÃO
    else if (message.startsWith("DN0CMD")) {
        int coordMode = message.substring(6).toInt();
        multiRobotCoord.setMode((CoordinationMode)coordMode);
        return;
    }
    
    // DEFINIR PAPEL
    else if (message.startsWith("DN0ROL")) {
        int role = message.substring(6).toInt();
        multiRobotCoord.setRole((RobotRole)role);
        return;
    }
    
    // DEFINIR LÍDER
    else if (message.startsWith("DN0LDR")) {
        String leader = message.substring(6);
        multiRobotCoord.setLeader(leader.c_str());
        return;
    }
    
    // OFFSET DE FORMAÇÃO
    else if (message.startsWith("DN0OFF")) {
        int xPos = message.indexOf('X');
        int yPos = message.indexOf('Y');
        if (xPos > 0 && yPos > xPos) {
            float offsetX = message.substring(xPos + 1, yPos).toFloat();
            float offsetY = message.substring(yPos + 1).toFloat();
            multiRobotCoord.setFormationOffset(offsetX, offsetY);
        }
        return;
    }
    
    // MODO AUTÔNOMO (ir para coordenada)
    else if (message.startsWith("DN0GOTO")) {
        // Formato: DN0GOTOX1.5Y2.0
        int xPos = message.indexOf('X');
        int yPos = message.indexOf('Y');
        if (xPos > 0 && yPos > xPos) {
            autoTargetX = message.substring(xPos + 1, yPos).toFloat() * 10; // converte para escala 0-9
            autoTargetY = message.substring(yPos + 1).toFloat() * 10;
            autoModeEnabled = true;
            Serial.printf("🎯 Modo autônomo ativado: destino X=%d, Y=%d\n", autoTargetX, autoTargetY);
        }
        return;
    }
    
    // PARAR MODO AUTÔNOMO
    else if (message == "DN0STOPAUTO") {
        autoModeEnabled = false;
        PWM_PID(0, 0);
        Serial.println("⏹️ Modo autônomo desativado");
        return;
    }
    
    // PARADA DE EMERGÊNCIA GLOBAL
    else if (message == "DN0EMERGENCY") {
        Serial.println("🚨 PARADA DE EMERGÊNCIA GLOBAL!");
        autoModeEnabled = false;
        pararCoreografia();
        PWM_PID(0, 0);
        desligaLeds();
        player.stop();
        publishMQTT("robot/emergency", "activated");
        return;
    }
    
    else {
        Serial.printf("⚠️ Comando não reconhecido: %s\n", message.c_str());
    }
}

static void parseMQTTPacket(uint8_t* data, size_t len) {
    if (len < 2) return;
    uint8_t packetType = (data[0] & 0xF0) >> 4;

    switch (packetType) {
        case 2: {  // CONNACK
            if (len < 4) {
                Serial.println("❌ CONNACK MQTT inválido");
                return;
            }
            uint8_t returnCode = data[3];
            if (returnCode == 0x00) {
                Serial.printf("✅ CARRINHO CONECTADO AO MQTT! ID=%s\n", robotId);
                mqttConnected = true;
                sendMQTTSubscribe("cmd", 1);
                //sendMQTTSubscribe("robot/+/pose", 2);
                //sendMQTTSubscribe("robot/+/avoidance", 3);
                //sendMQTTSubscribe("robot/+/coord_mode", 4);

                sendMQTTSubscribe("robot/pose/+", 2);
                sendMQTTSubscribe("robot/avoidance", 3);
                sendMQTTSubscribe("robot/coord_mode", 4);


                sendMQTTSubscribe("robot/emergency", 5);
                // Publica status inicial
                publishMQTT("robot/status", "online");
                publishMQTT("robot/version", version);
            } else {
                Serial.printf("❌ MQTT recusou a conexão (CONNACK 0x%02X)\n", returnCode);
            }
            break;
        }
        case 3: {  // PUBLISH
            int pos = 1;
            int remainingLength = 0;
            int multiplier = 1;
            while (pos < (int)len) {
                uint8_t b = data[pos++];
                remainingLength += (b & 0x7F) * multiplier;
                multiplier *= 128;
                if (!(b & 0x80)) break;
            }
            if (pos + 2 > (int)len) return;
            uint16_t topicLen = (data[pos] << 8) | data[pos + 1];
            pos += 2;
            if (pos + topicLen > (int)len) return;
            String topic = String((char*)(data + pos), topicLen);
            pos += topicLen;
            uint8_t qos = (data[0] & 0x06) >> 1;
            if (qos > 0) pos += 2;
            int payloadLen = remainingLength - 2 - topicLen - (qos > 0 ? 2 : 0);
            if (payloadLen < 0 || pos + payloadLen > (int)len) return;
            String payload = String((char*)(data + pos), payloadLen);
            
            if (topic == "cmd") {
                processMQTTCommand(payload);
            } 
            else if (topic.startsWith("robot/pose/")) {
                // Processa pose de outro robô com covariância
                StaticJsonDocument<512> doc;
                DeserializationError error = deserializeJson(doc, payload);
                if (!error) {
                    String otherId = topic.substring(11);
                    float otherX = doc["x"];
                    float otherY = doc["y"];
                    float otherTheta = doc["theta"] | 0;
                    float cov_xx = doc["cov_xx"] | 0.01f;
                    float cov_yy = doc["cov_yy"] | 0.01f;
                    float otherVx = doc["vx"] | 0;
                    float otherVy = doc["vy"] | 0;
                    
                    // Atualiza sistema de evitação
                    collisionAvoidance.updateOtherRobots(
                        otherId, otherX, otherY, otherTheta, cov_xx, cov_yy, otherVx, otherVy);
                }
            }
            else if (topic.startsWith("robot/avoidance/")) {
                // Processa status de evitação de outros robôs (para coordenação)
                StaticJsonDocument<256> doc;
                deserializeJson(doc, payload);
                // Log para debug
                String otherId = topic.substring(16, topic.indexOf('/', 16));
                Serial.printf("📡 [%s] Evitação: risco=%d\n", otherId.c_str(), doc["risk_level"] | 0);
            }
            break;
        }
        case 13:  // PINGRESP
            Serial.println("🏓 PINGRESP");
            break;
        case 9:   // SUBACK
            Serial.println("✅ SUBACK");
            break;
        default:
            break;
    }
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("❌ Carrinho desconectado do WebSocket/MQTT; tentando reconectar...");
            mqttConnected = false;
            autoModeEnabled = false;
            break;
        case WStype_CONNECTED:
            Serial.println("🔗 WebSocket conectado; autenticando no MQTT...");
            sendMQTTConnect();
            break;
        case WStype_BIN:
            parseMQTTPacket(payload, length);
            break;
        default:
            Serial.printf("ℹ️ Evento WebSocket: %d\n", type);
            break;
    }
}

void initMQTTWebSocket() {
    Serial.printf("🌐 Conectando o carrinho ao broker WSS: %s:%d%s\n", mqtt_server, mqtt_port, mqtt_path);
    webSocket.setExtraHeaders("Sec-WebSocket-Protocol: mqtt");
    webSocket.setReconnectInterval(5000);
    webSocket.beginSSL(mqtt_server, mqtt_port, mqtt_path);
    webSocket.onEvent(webSocketEvent);
}

void mqttLoop() {
    webSocket.loop();
    atualizarCoreografia();
    
    unsigned long now = millis();
    
    if (mqttConnected) {
        // Ping
        if (now - lastPing > PING_INTERVAL_MS) {
            lastPing = now;
            sendMQTTPingReq();
        }
        
        // Publica pose do robô com covariância
        if (now - lastPosePublish > POSE_PUBLISH_INTERVAL_MS) {
            lastPosePublish = now;
            
            // Atualiza odometria
            updateOdometry();
            
            // Corrige com giroscópio
            localization.correctOdometryWithGyro();
            
            // Publica pose com covariância
            localization.publishPoseWithCovariance();
        }
        
        // Verifica e evita colisões (alta frequência)
        if (now - lastAvoidanceCheck > AVOIDANCE_CHECK_INTERVAL_MS) {
            lastAvoidanceCheck = now;
            
            // Remove robôs antigos
            collisionAvoidance.removeStaleRobots();
            
            // Verifica nível de risco
            String nearestRobot;
            float nearestDist;
            int riskLevel = collisionAvoidance.checkCollisionRisk(nearestDist, nearestRobot);
            
            if (riskLevel == 2) {
                // Risco crítico: para emergência
                collisionAvoidance.emergencyStop();
                autoModeEnabled = false;
            } 
            else if (!autoModeEnabled && !coreografiaEmExecucao) {
                // Apenas em modo manual, aplica correção de evitação
                int avoidX, avoidY;
                collisionAvoidance.getAvoidanceCommand(avoidX, avoidY);
                
                if (avoidX != 0 || avoidY != 0) {
                    // Aplica correção de movimento se houver perigo
                    PWM_PID(avoidX, avoidY);
                    // Pequeno delay para não sobrecarregar
                    delay(20);
                }
            }
            else if (autoModeEnabled && !coreografiaEmExecucao) {
                // Modo autônomo: navega para destino
                int errorX = autoTargetX - (int)(robotX * 10);
                int errorY = autoTargetY - (int)(robotY * 10);
                
                if (abs(errorX) < 2 && abs(errorY) < 2) {
                    // Chegou ao destino
                    autoModeEnabled = false;
                    PWM_PID(0, 0);
                    Serial.println("✅ Destino alcançado!");
                    publishMQTT("robot/status", "destination_reached");
                } else {
                    // Calcula comando proporcional
                    int cmdX = constrain(errorX / 5, -9, 9);
                    int cmdY = constrain(errorY / 5, -9, 9);
                    
                    // Aplica evitação de colisão mesmo em modo autônomo
                    int avoidX, avoidY;
                    collisionAvoidance.getAvoidanceCommand(avoidX, avoidY);
                    
                    cmdX = constrain(cmdX + avoidX, -9, 9);
                    cmdY = constrain(cmdY + avoidY, -9, 9);
                    
                    PWM_PID(cmdX, cmdY);
                }
            }
        }
        
        // Publica status periódico
        if (now - lastStatusPublish > STATUS_PUBLISH_INTERVAL_MS) {
            lastStatusPublish = now;
            collisionAvoidance.publishAvoidanceStatus();
            multiRobotCoord.publishModeStatus();
            
            // Log de debug
            int activeRobots = collisionAvoidance.getActiveRobotsCount();
            if (activeRobots > 0) {
                collisionAvoidance.printStatus();
            }
        }
    }
}

#endif
