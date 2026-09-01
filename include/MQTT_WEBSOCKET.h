#ifndef MQTT_WEBSOCKET_H
#define MQTT_WEBSOCKET_H

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
extern char id_robo[];
extern float roboX, roboY, roboTheta;

// Variáveis estáticas adicionais
static unsigned long ultimoPing = 0;
static unsigned long ultimaPosePublish = 0;
static unsigned long ultimoStatusPublish = 0;
static unsigned long ultima_checagem_desvio = 0;
static const unsigned long INTERVALO_PING_MS = 30000;
static const unsigned long INTERVALO_POSE_PUBLISH_MS = 100;      // 10Hz
static const unsigned long INTERVALO_STATUS_PUBLISH_MS = 5000;   // 5 segundos
static const unsigned long INTERVALO_CHECAGEM_DESVIO_MS = 50;    // 20Hz

static bool autonomo_disponivel = false; //flag controle autônomo
static int auto_alvoX = 0, auto_alvoY = 0;

// Forward declarations
void coreo();

// Função para publicar MQTT
void publicarMQTT(const char* topico, const char* mensagem) {
    if (!mqttConnected) return;
    
    uint8_t pacote[1024];  // Pacote MQTTpub Aumentado para comportar JSON maior
    int pos = 0;
    
    pacote[pos++] = 0x30; // PUBLISH QoS 0
    
    // Calcula tamanho do mensagem
    int topico_tamanho = strlen(topico);
    int mensagem_tamanho = strlen(mensagem);
    int restante_tamanho = 2 + topico_tamanho + mensagem_tamanho;
    
    // Encode remaining tamanho
    int pos_restante_tamanho = pos;
    pos++;
    if (restante_tamanho < 128) {
        pacote[pos_restante_tamanho] = restante_tamanho;
    } else {
        pacote[pos_restante_tamanho] = (restante_tamanho % 128) | 0x80;
        pacote[pos++] = restante_tamanho / 128;
    }
    
    // Tamanho do tópico
    pacote[pos++] = (topico_tamanho >> 8) & 0xFF;
    pacote[pos++] = topico_tamanho & 0xFF;
    
    // Tópico
    memcpy(pacote + pos, topico, topico_tamanho);
    pos += topico_tamanho;
    
    // Mensagem
    memcpy(pacote + pos, mensagem, mensagem_tamanho);
    pos += mensagem_tamanho;
    
    webSocket.sendBIN(pacote, pos);
}

static int escreverTamanhoRestante(uint8_t* buffer, int tamanho) {
    int indice = 0;
    do {
        uint8_t byteCodificado = tamanho % 128;
        tamanho = tamanho / 128;
        if (tamanho > 0) {
            byteCodificado |= 0x80;
        }
        buffer[indice++] = byteCodificado;
    } while (tamanho > 0);
    return indice;
}

static int escreverStringMQTT(uint8_t* buffer, const char* str) {
    uint16_t tamanho = strlen(str);
    buffer[0] = (tamanho >> 8) & 0xFF;
    buffer[1] = tamanho & 0xFF;
    memcpy(buffer + 2, str, tamanho);
    return 2 + tamanho;
}

static void enviarMQTTConnect() {
    String id_cliente = String(id_robo) + "_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    const char* id_clt = id_cliente.c_str();

    Serial.printf("📤 MQTT CONECTADO: cliente=%s, broker=%s:%d\n", id_clt, mqtt_server, mqtt_port);

    uint8_t mensagem[256];
    int indice_mensagem = 0;

    indice_mensagem += escreverStringMQTT(mensagem + indice_mensagem, "MQTT");
    mensagem[indice_mensagem++] = 0x04;
    uint8_t flags_conectadas = 0x02; //uint8_t: 0 até 255
    bool comUsuario = (mqtt_user != NULL && strlen(mqtt_user) > 0);
    bool comSenha = (mqtt_password != NULL && strlen(mqtt_password) > 0);
    if (comUsuario) {
        flags_conectadas = flags_conectadas | 0x80; 
    }
    if (comSenha) {
        flags_conectadas = flags_conectadas | 0x40;
    }

    mensagem[indice_mensagem++] = flags_conectadas;
    mensagem[indice_mensagem++] = 0x00;
    mensagem[indice_mensagem++] = 0x3C;
    indice_mensagem += escreverStringMQTT(mensagem + indice_mensagem, id_clt);
    if (comUsuario) {
        indice_mensagem += escreverStringMQTT(mensagem + indice_mensagem, mqtt_user);
    }
    if (comSenha) {
        indice_mensagem += escreverStringMQTT(mensagem + indice_mensagem, mqtt_password);
    }

    uint8_t header_fixo[5];
    header_fixo[0] = 0x10;
    int tamanho_tamanho_restante = escreverTamanhoRestante(header_fixo + 1, indice_mensagem);

    uint8_t pacote[300];
    memcpy(pacote, header_fixo, 1 + tamanho_tamanho_restante);
    memcpy(pacote + 1 + tamanho_tamanho_restante, mensagem, indice_mensagem);

    webSocket.sendBIN(pacote, 1 + tamanho_tamanho_restante + indice_mensagem);
}

static void enviarMQTTSubscribe(const char* topico, uint16_t packetId) {
    uint8_t mensagem[128];
    int indice_mensagem = 0;
    mensagem[indice_mensagem++] = (packetId >> 8) & 0xFF;
    mensagem[indice_mensagem++] = packetId & 0xFF;
    indice_mensagem += escreverStringMQTT(mensagem + indice_mensagem, topico);
    mensagem[indice_mensagem++] = 0x00;

    uint8_t header_fixo[5];
    header_fixo[0] = 0x82;
    int tamanho_tamanho_restante = escreverTamanhoRestante(header_fixo + 1, indice_mensagem);

    uint8_t pacote[200];
    memcpy(pacote, header_fixo, 1 + tamanho_tamanho_restante);
    memcpy(pacote + 1 + tamanho_tamanho_restante, mensagem, indice_mensagem);
    webSocket.sendBIN(pacote, 1 + tamanho_tamanho_restante + indice_mensagem);
    Serial.printf("📥 Inscrevendo no tópico MQTT: %s\n", topico);
}

static void enviarMQTTRequisicaoPing() {
    uint8_t pacote[2] = {0xC0, 0x00};
    webSocket.sendBIN(pacote, 2);
}

// Processa comandos recebidos via MQTT
void processarMQTTComando(const String& mensagem) {
    Serial.printf("🎮 Comando: %s\n", mensagem.c_str());

    if (!mensagem.startsWith("DN0")) {
        Serial.println("⚠️ Comando inválido");
        return;
    }

    // PARAR
    if (mensagem == "DN0CPA") {
        Serial.println("🛑 PARAR");
        autonomo_disponivel = false;
        pararCoreografia();
        PWM_PID(0, 0);
        desligaLeds();
        return;
    }
    
    // COREOGRAFIA
    else if (mensagem == "DN0CG") {
        Serial.println("🎬 INICIAR COREOGRAFIA");
        iniciarCoreografia();
        return;
    }
    
    // LEDS
    else if (mensagem.startsWith("DN0CL")) {
        int ledNum = mensagem.substring(5).toInt();
        if (ledNum >= 0 && ledNum < 8) {
            digitalWrite(led[ledNum], HIGH);
            Serial.printf("💡 LED %d ON\n", ledNum);
        }
        return;
    }
    else if (mensagem.startsWith("DN0CD")) {
        int ledNum = mensagem.substring(5).toInt();
        if (ledNum >= 0 && ledNum < 8) {
            digitalWrite(led[ledNum], LOW);
            Serial.printf("💡 LED %d OFF\n", ledNum);
        }
        return;
    }
    
    // MÚSICAS
    else if (mensagem.startsWith("DN0CM")) {
        int musicaId = mensagem.substring(5).toInt();
        tocarMusica(musicaId);
        return;
    }
    else if (mensagem == "DN0CPS") {
        pararMusica();
        return;
    }
    
    // MOVIMENTO
    else if (mensagem.indexOf('X') != -1 && mensagem.indexOf('Y') != -1) {
        if (coreografiaEmExecucao) {
            pararCoreografia();
        }
        
        autonomo_disponivel = false;
        
        int xPos = mensagem.indexOf('X');
        int yPos = mensagem.indexOf('Y');
        char xDir = mensagem.charAt(xPos + 1);
        char yDir = mensagem.charAt(yPos + 1);
        int velocX = mensagem.charAt(xPos + 2) - '0';
        int velocY = mensagem.charAt(yPos + 2) - '0';
        if (xDir == '-') velocX = -velocX;
        if (yDir == '-') velocY = -velocY;
        
        PWM_PID(velocX, velocY);
        Serial.printf("🕹️ Movimento: X=%d, Y=%d\n", velocX, velocY);
        return;
    }
    
    // ===== COMANDOS GEOMÉTRICOS =====
    
    // QUADRADO
    else if (mensagem.startsWith("DN0QUADRD")) {
        float lado = (mensagem.length() > 6) ? mensagem.substring(6).toFloat() : 0.5;
        desenharQuadrado(lado);
        char buffer[30];
        sprintf(buffer, "Quadrado %.2fm", lado);
        publicarMQTT("robo/status", buffer);
        return;
    }
    
    // TRIÂNGULO
    else if (mensagem.startsWith("DN0TRIANG")) {
        float lado = (mensagem.length() > 6) ? mensagem.substring(6).toFloat() : 0.5;
        desenharTrianguloEquilatero(lado);
        char buffer[30];
        sprintf(buffer, "Triângulo %.2fm", lado);
        publicarMQTT("robo/status", buffer);
        return;
    }
    
    // CORAÇÃO
    else if (mensagem.startsWith("DN0COR")) {
        float tamanho = (mensagem.length() > 6) ? mensagem.substring(6).toFloat() : 0.5;
        desenharCoracao(tamanho);
        char buffer[30];
        sprintf(buffer, "Coração %.2fm", tamanho);
        publicarMQTT("robo/status", buffer);
        return;
    }
    
    // ANDAR DISTÂNCIA
    else if (mensagem.startsWith("DN0FRENTEP")) {
        float distancia = mensagem.substring(7).toFloat();
        movFrentePreciso(distancia);
        char buffer[30];
        sprintf(buffer, "Andou %.2fm", distancia);
        publicarMQTT("robo/status", buffer);
        return;
    }
    
    // GIRAR GRAUS
    else if (mensagem.startsWith("DN0GIRAR")) {
        float angulo = mensagem.substring(7).toFloat();
        girarGraus(angulo);
        char buffer[30];
        sprintf(buffer, "Girou %.1f°", angulo);
        publicarMQTT("robo/status", buffer);
        return;
    }
    
    // POSIÇÃO (NOVO)
    else if (mensagem == "DN0POS") {
        char buffer[100];
        sprintf(buffer, "{\"x\":%.3f,\"y\":%.3f,\"theta\":%.2f}", 
                roboX, roboY, roboTheta * 180.0 / PI);
        publicarMQTT("robo/posicao", buffer);
        return;
    }
    
    //NOVO ID ROBO
    else if (mensagem.startsWith("DN0ID")) {
        String novoId = mensagem.substring(5);
        salvarIdRobo(novoId.c_str());
        publicarMQTT("robo/status", "ID atualizado");
        return;
    }
    
    // RESETAR ODOMETRIA
    else if (mensagem == "DN0RSTODO") {
        resetarOdometria();
        publicarMQTT("robo/status", "Odometria resetada");
        return;
    }
    
    //========= OUTROS .h ==============
    // MODO DE EVITAÇÃO
    else if (mensagem.startsWith("DN0AVD")) {
        int mode = mensagem.substring(6).toInt();
        collisionAvoidance.setMode((AvoidanceMode)mode);
        char buffer[50];
        sprintf(buffer, "{\"mode\":%d}", mode);
        publicarMQTT("robo/avoidance/mode", buffer);
        return;
    }
    
    // RAIO DE SEGURANÇA
    else if (mensagem.startsWith("DN0RAD")) {
        float radius = mensagem.substring(6).toFloat();
        collisionAvoidance.setSafetyRadius(radius);
        char buffer[50];
        sprintf(buffer, "{\"radius\":%.2f}", radius);
        publicarMQTT("robo/avoidance/radius", buffer);
        return;
    }
    
    // MODO DE COORDENAÇÃO
    else if (mensagem.startsWith("DN0CMD")) {
        int coordMode = mensagem.substring(6).toInt();
        multiRobotCoord.setMode((CoordinationMode)coordMode);
        return;
    }
    
    // DEFINIR PAPEL
    else if (mensagem.startsWith("DN0ROL")) {
        int role = mensagem.substring(6).toInt();
        multiRobotCoord.setRole((RobotRole)role);
        return;
    }
    
    // DEFINIR LÍDER
    else if (mensagem.startsWith("DN0LDR")) {
        String leader = mensagem.substring(6);
        multiRobotCoord.setLeader(leader.c_str());
        return;
    }
    
    // OFFSET DE FORMAÇÃO
    else if (mensagem.startsWith("DN0OFFSET")) {
        int xPos = mensagem.indexOf('X');
        int yPos = mensagem.indexOf('Y');
        if (xPos > 0 && yPos > xPos) {
            float offsetX = mensagem.substring(xPos + 1, yPos).toFloat();
            float offsetY = mensagem.substring(yPos + 1).toFloat();
            multiRobotCoord.setFormationOffset(offsetX, offsetY);
        }
        return;
    }
    
    // MODO AUTÔNOMO (ir para coordenada)
    else if (mensagem.startsWith("DN0AUTO")) {
        // Formato: DN0GOTOX1.5Y2.0
        int xPos = mensagem.indexOf('X');
        int yPos = mensagem.indexOf('Y');
        if (xPos > 0 && yPos > xPos) {
            auto_alvoX = mensagem.substring(xPos + 1, yPos).toFloat() * 10; // converte para escala 0-9
            auto_alvoY = mensagem.substring(yPos + 1).toFloat() * 10;
            autonomo_disponivel = true;
            Serial.printf("🎯 Modo autônomo ativado: destino X=%d, Y=%d\n", auto_alvoX, auto_alvoY);
        }
        return;
    }
    
//========================================================


    // PARAR MODO AUTÔNOMO
    else if (mensagem == "DN0PARARAUTO") {
        autonomo_disponivel = false;
        PWM_PID(0, 0);
        Serial.println("⏹️ Modo autônomo desativado");
        return;
    }
    
    // PARADA DE EMERGÊNCIA GLOBAL
    else if (mensagem == "DN0192") {
        Serial.println("🚨 PARADA DE EMERGÊNCIA!");
        autonomo_disponivel = false;
        pararCoreografia();
        PWM_PID(0, 0);
        desligaLeds();
        player.stop();
        publicarMQTT("robo/emergencia", "ativado");
        return;
    }
    
    else {
        Serial.printf("⚠️ Comando não reconhecido: %s\n", mensagem.c_str());
    }
}

//analisar os pacotes recebidos MQTT
static void analisarMQTTPacote(uint8_t* dado, size_t tamanho) {
    if (tamanho < 2) return;
    uint8_t pacote_tipo = (dado[0] & 0xF0) >> 4;

    switch (pacote_tipo) {
        case 2: {  // CONNACK
            if (tamanho < 4) {
                Serial.println("❌ CONNACK MQTT inválido");
                return;
            }
            uint8_t retornarCodigo = dado[3];
            if (retornarCodigo == 0x00) {
                Serial.printf("✅ CARRINHO CONECTADO AO MQTT! ID=%s\n", id_robo);
                mqttConnected = true;
                enviarMQTTSubscribe("cmd", 1);
                enviarMQTTSubscribe("robo/pose/+", 2);
                enviarMQTTSubscribe("robo/avoidance", 3);
                enviarMQTTSubscribe("robo/coord_mode", 4);
                enviarMQTTSubscribe("robo/emergencia", 5);
                // Publica status inicial
                publicarMQTT("robo/status", "online");
                publicarMQTT("robo/version", version);
            } else {
                Serial.printf("❌ MQTT recusou a conexão (CONNACK 0x%02X)\n", retornarCodigo);
            }
            break;
        }
        case 3: {  // PUBLISH
            int pos = 1;
            int restante_tamanho = 0;
            int multiplicador = 1;

            while (pos < (int)tamanho) {
                uint8_t proximo_byte = dado[pos++];
                restante_tamanho += (proximo_byte & 0x7F) * multiplicador; //0x7F pega só os 7 primeiros dígitos
                multiplicador *= 128;
                if (!(proximo_byte & 0x80)) break; //se o 8º byte for 0, sai do loop
            }

            if (pos + 2 > (int)tamanho) return;
            uint16_t topico_tamanho = (dado[pos] << 8) | dado[pos + 1]; // Lê 2 bytes e junta para formar o tamanho 
            pos += 2;

            if (pos + topico_tamanho > (int)tamanho) return;
            String topico = String((char*)(dado + pos), topico_tamanho);
            pos += topico_tamanho;
            uint8_t qos = (dado[0] & 0x06) >> 1;
            if (qos > 0) pos += 2;
            int mensagem_tamanho = restante_tamanho - 2 - topico_tamanho - (qos > 0 ? 2 : 0);
            if (mensagem_tamanho < 0 || pos + mensagem_tamanho > (int)tamanho) return;
            String mensagem = String((char*)(dado + pos), mensagem_tamanho);
            
            if (topico == "cmd") {
                processarMQTTComando(mensagem);
            } 
            else if (topico.startsWith("robo/pose/")) {
                // Processa pose de outro robô com covariância
                StaticJsonDocument<512> doc; //documento JSON com espaço de 512bytes
                DeserializationError error = deserializeJson(doc, mensagem);
                if (!error) { //verifica se converteu para json
                    String outroId = topico.substring(11);
                    float outroX = doc["x"];
                    float outroY = doc["y"];
                    float outroTheta = doc["theta"] | 0;
                    float cov_xx = doc["cov_xx"] | 0.01f;
                    float cov_yy = doc["cov_yy"] | 0.01f;
                    float outroVx = doc["vx"] | 0;
                    float outroVy = doc["vy"] | 0;
                    
                    // Atualiza sistema de evitação
                    collisionAvoidance.updateOtherRobots(
                        outroId, outroX, outroY, outroTheta, cov_xx, cov_yy, outroVx, outroVy);
                }
            }
            else if (topico.startsWith("robo/avoidance/")) {
                // Processa status de evitação de outros robôs (para coordenação)
                StaticJsonDocument<256> doc;
                deserializeJson(doc, mensagem);
                // Log para debug
                String outroId = topico.substring(16, topico.indexOf('/', 16));
                Serial.printf("📡 [%s] Evitação: risco=%d\n", outroId.c_str(), doc["risk_level"] | 0);
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

// gerenciar os eventos da conexao WebSocket (conexão, recebimento dos dados, etc)
void webSocketEventos(WStype_t tipo, uint8_t* mensagem, size_t tamanho) {
    switch (tipo) {
        case WStype_DISCONNECTED:
            Serial.println("❌ Carrinho desconectado do WebSocket/MQTT; tentando reconectar...");
            mqttConnected = false;
            autonomo_disponivel = false;
            break;
        case WStype_CONNECTED:
            Serial.println("🔗 WebSocket conectado; autenticando no MQTT...");
            enviarMQTTConnect();
            break;
        case WStype_BIN:
            analisarMQTTPacote(mensagem, tamanho);
            break;
        default:
            Serial.printf("ℹ️ Evento WebSocket: %d\n", tipo);
            break;
    }
}


//inicia a conexao WebSocket
void iniciarMQTTWebSocket() { 
    Serial.printf("🌐 Conectando o carrinho ao broker WSS: %s:%d%s\n", mqtt_server, mqtt_port, mqtt_path);
    webSocket.setExtraHeaders("Sec-WebSocket-Protocol: mqtt");
    webSocket.setReconnectInterval(5000);
    webSocket.beginSSL(mqtt_server, mqtt_port, mqtt_path);
    webSocket.onEvent(webSocketEventos);
}

void mqttLoop() {
    webSocket.loop();
    atualizarCoreografia();
    
    unsigned long temp_atual = millis();
    
    if (mqttConnected) {
        // Ping
        if (temp_atual - ultimoPing > INTERVALO_PING_MS) {
            ultimoPing = temp_atual;
            enviarMQTTRequisicaoPing();
        }
        
        // Publica pose do robô com covariância
        if (temp_atual - ultimaPosePublish > INTERVALO_POSE_PUBLISH_MS) {
            ultimaPosePublish = temp_atual;
            
            // Atualiza odometria
            atualizarOdometria();
            
            // Corrige com giroscópio
            localization.correctOdometryWithGyro();
            
            // Publica pose com covariância
            localization.publishPoseWithCovariance();
        }
        
        // Verifica e evita colisões (alta frequência)
        if (temp_atual - ultima_checagem_desvio > INTERVALO_CHECAGEM_DESVIO_MS) {
            ultima_checagem_desvio = temp_atual;
            
            // Remove robôs antigos
            collisionAvoidance.removeStaleRobots();
            
            // Verifica nível de risco
            String nearestRobot;
            float nearestDist;
            int riskLevel = collisionAvoidance.checkCollisionRisk(nearestDist, nearestRobot);
            
            if (riskLevel == 2) {
                // Risco crítico: para emergência
                collisionAvoidance.emergencyStop();
                autonomo_disponivel = false;
            } 
            else if (!autonomo_disponivel && !coreografiaEmExecucao) {
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
            else if (autonomo_disponivel && !coreografiaEmExecucao) {
                // Modo autônomo: navega para destino
                int errorX = auto_alvoX - (int)(roboX * 10);
                int errorY = auto_alvoY - (int)(roboY * 10);
                
                if (abs(errorX) < 2 && abs(errorY) < 2) {
                    // Chegou ao destino
                    autonomo_disponivel = false;
                    PWM_PID(0, 0);
                    Serial.println("✅ Destino alcançado!");
                    publicarMQTT("robo/status", "destino_alcancado");
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
        if (temp_atual - ultimoStatusPublish > INTERVALO_STATUS_PUBLISH_MS) {
            ultimoStatusPublish = temp_atual;
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