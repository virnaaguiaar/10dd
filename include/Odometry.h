#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "Configs.h"
#include <ArduinoJson.h>

// Implemented by MQTTWebSocketCorrect.h. This declaration must appear before
// the localization and avoidance headers use it.
void publishMQTT(const char* topic, const char* payload);

extern float robotX, robotY, robotTheta;
extern float robotVx, robotVy, robotVtheta;
extern volatile int pulsosEncoderE, pulsosEncoderD;
extern volatile int lastPulsosEncoderE, lastPulsosEncoderD;
extern unsigned long lastOdometryTime;
extern char robotId[];
extern WebSocketsClient webSocket;

// Atualiza a odometria baseado nos encoders
void updateOdometry() {
  unsigned long now = millis();
  float dt = (now - lastOdometryTime) / 1000.0;
  if (dt <= 0 || dt > 0.1) {
    lastOdometryTime = now;
    return;
  }
  
  // Calcula deslocamento das rodas
  int deltaE = pulsosEncoderE - lastPulsosEncoderE;
  int deltaD = pulsosEncoderD - lastPulsosEncoderD;
  
  float distE = deltaE * DIST_POR_PULSO;
  float distD = deltaD * DIST_POR_PULSO;
  
  // Velocidades lineares das rodas (m/s)
  float vE = distE / dt;
  float vD = distD / dt;
  
  // Velocidade linear e angular do robô
  float v = (vE + vD) / 2.0;
  float w = (vD - vE) / BASE_RODAS;
  
  robotVx = v * cos(robotTheta);
  robotVy = v * sin(robotTheta);
  robotVtheta = w;
  
  // Atualiza posição (modelo diferencial)
  if (fabs(w) < 0.01) {
    // Movimento quase retilíneo
    robotX += v * cos(robotTheta) * dt;
    robotY += v * sin(robotTheta) * dt;
  } else {
    // Movimento curvilíneo
    float r = v / w;
    float deltaTheta = w * dt;
    robotX += r * (sin(robotTheta + deltaTheta) - sin(robotTheta));
    robotY += r * (-cos(robotTheta + deltaTheta) + cos(robotTheta));
    robotTheta += deltaTheta;
  }
  
  // Normaliza ângulo
  while (robotTheta > PI) robotTheta -= 2 * PI;
  while (robotTheta < -PI) robotTheta += 2 * PI;
  
  // Guarda valores para próxima iteração
  lastPulsosEncoderE = pulsosEncoderE;
  lastPulsosEncoderD = pulsosEncoderD;
  lastOdometryTime = now;
}

// Publica posição via MQTT para coordenação
void publishRobotPose() {
  StaticJsonDocument<256> doc;
  doc["robot_id"] = robotId;
  doc["x"] = robotX;
  doc["y"] = robotY;
  doc["theta"] = robotTheta;
  doc["vx"] = robotVx;
  doc["vy"] = robotVy;
  doc["vtheta"] = robotVtheta;
  doc["timestamp"] = millis() / 1000.0;
  
  char buffer[256];
  serializeJson(doc, buffer);
  
  String topic = String("robot/pose/") + robotId;
  // Envia via MQTT (precisa ser implementado no MQTTWebSocketCorrect.h)
  publishMQTT(topic.c_str(), buffer);
}

// Reseta odometria (útil para calibração)
void resetOdometry() {
  robotX = 0;
  robotY = 0;
  robotTheta = 0;
  robotVx = 0;
  robotVtheta = 0;
  resetEncoders();
  Serial.println("📍 Odometria resetada");
}

#endif
