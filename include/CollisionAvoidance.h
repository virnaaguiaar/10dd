#ifndef COLLISION_AVOIDANCE_H
#define COLLISION_AVOIDANCE_H

#include "Configs.h"
#include "Odometry.h"
#include <map>
#include <vector>

void PWM(int dirX, int dirY);

// Modos de evitação
enum AvoidanceMode {
  AVOID_OFF = 0,
  AVOID_SIMPLE,      // Evita apenas quando muito próximo
  AVOID_VELOCITY,    // Modifica velocidades suavemente
  AVOID_POTENTIAL    // Campo potencial
};

// Estrutura para informações de outros robôs
struct OtherRobot {
  char id[16];
  float x, y, theta;
  float vx, vy, vtheta;
  float cov_xx, cov_yy, cov_tt;
  unsigned long lastUpdate;
  float distance;
  float timeToCollision;
  
  // Para predição de trajetória
  std::vector<float> predictedX;
  std::vector<float> predictedY;
};

class CollisionAvoidance {
private:
  std::map<String, OtherRobot> otherRobots;
  AvoidanceMode mode = AVOID_SIMPLE;
  
  // Parâmetros de segurança
  float safetyRadius = 0.25f;      // Raio de segurança (metros)
  float warningRadius = 0.4f;      // Raio de alerta
  float criticalRadius = 0.15f;    // Raio crítico (para em emergência)
  
  // Parâmetros para campo potencial
  float attractiveGain = 1.0f;
  float repulsiveGain = 10.0f;
  float influenceRadius = 1.0f;
  
  // Filtros
  float lastAvoidanceX = 0, lastAvoidanceY = 0;
  unsigned long lastAvoidTime = 0;
  
public:
  void setMode(AvoidanceMode newMode) {
    mode = newMode;
    Serial.printf("🛡️ Modo de evitação alterado para: %d\n", mode);
  }
  
  void setSafetyRadius(float radius) {
    safetyRadius = radius;
    warningRadius = radius * 1.6f;
    criticalRadius = radius * 0.6f;
  }
  
  void updateOtherRobots(const String& robotId, float x, float y, float theta,
                         float cov_xx, float cov_yy, float vx = 0, float vy = 0) {
    if (robotId == String(::robotId)) return;  // Ignora a si mesmo
    
    OtherRobot& other = otherRobots[robotId];
    strcpy(other.id, robotId.c_str());
    other.x = x;
    other.y = y;
    other.theta = theta;
    other.vx = vx;
    other.vy = vy;
    other.cov_xx = cov_xx;
    other.cov_yy = cov_yy;
    other.lastUpdate = millis();
    
    // Calcula distância
    float dx = x - robotX;
    float dy = y - robotY;
    other.distance = sqrt(dx*dx + dy*dy);
    
    // Prediz trajetória (simples: movimento linear)
    other.predictedX.clear();
    other.predictedY.clear();
    for (float t = 0.1; t <= 1.0; t += 0.1) {
      other.predictedX.push_back(x + other.vx * t);
      other.predictedY.push_back(y + other.vy * t);
    }
    
    // Calcula tempo para colisão (assumindo velocidades constantes)
    float relVx = robotVx - other.vx;
    float relVy = robotVy - other.vy;
    float relSpeed = sqrt(relVx*relVx + relVy*relVy);
    if (relSpeed > 0.01) {
      other.timeToCollision = other.distance / relSpeed;
    } else {
      other.timeToCollision = 999.0f;
    }
  }
  
  void removeStaleRobots(unsigned long timeoutMs = 2000) {
    auto it = otherRobots.begin();
    while (it != otherRobots.end()) {
      if (millis() - it->second.lastUpdate > timeoutMs) {
        Serial.printf("🗑️ Removendo robô %s (stale)\n", it->first.c_str());
        it = otherRobots.erase(it);
      } else {
        ++it;
      }
    }
  }
  
  // Verifica se há perigo iminente
  int checkCollisionRisk(float& nearestDistance, String& nearestRobot) {
    int risk = 0;  // 0=safe, 1=warning, 2=critical
    nearestDistance = 999.0f;
    
    for (auto& pair : otherRobots) {
      OtherRobot& other = pair.second;
      
      if (other.distance < nearestDistance) {
        nearestDistance = other.distance;
        nearestRobot = other.id;
      }
      
      if (other.distance < criticalRadius) {
        risk = 2;  // Risco crítico
      } else if (other.distance < warningRadius && risk < 2) {
        risk = 1;  // Alerta
      }
      
      // Verifica tempo para colisão
      if (other.timeToCollision < 1.0f && other.distance < safetyRadius) {
        risk = 2;
      }
    }
    
    return risk;
  }
  
  // Gera comando de evitação baseado em campo potencial
  void getAvoidanceCommand(int& outX, int& outY) {
    outX = 0;
    outY = 0;
    
    if (mode == AVOID_OFF) return;
    
    // Campo potencial repulsivo para cada robô próximo
    for (auto& pair : otherRobots) {
      OtherRobot& other = pair.second;
      
      if (other.distance < influenceRadius) {
        // Vetor direção do obstáculo
        float dx = robotX - other.x;
        float dy = robotY - other.y;
        float dist = other.distance;
        
        if (dist < 0.01) dist = 0.01;
        
        // Força repulsiva (inversamente proporcional ao quadrado da distância)
        float magnitude = repulsiveGain * (1.0f / (dist * dist));
        
        // Limita magnitude máxima
        if (magnitude > 5.0f) magnitude = 5.0f;
        
        // Direção normalizada
        float nx = dx / dist;
        float ny = dy / dist;
        
        if (mode == AVOID_POTENTIAL) {
          outX += (int)(nx * magnitude * 10);
          outY += (int)(ny * magnitude * 10);
        } else if (mode == AVOID_SIMPLE && dist < safetyRadius) {
          // Evitação simples: para e vira
          if (fabs(nx) > fabs(ny)) {
            outX = (nx > 0) ? 5 : -5;
            outY = (ny > 0) ? 3 : -3;
          } else {
            outX = (nx > 0) ? 3 : -3;
            outY = (ny > 0) ? 5 : -5;
          }
          break;
        }
      }
    }
    
    // Modo velocidade: suaviza as mudanças
    if (mode == AVOID_VELOCITY) {
      unsigned long now = millis();
      float dt = (now - lastAvoidTime) / 1000.0f;
      if (dt > 0.05) dt = 0.05;
      
      // Filtro de primeira ordem
      float alpha = 0.7f;
      outX = alpha * outX + (1 - alpha) * lastAvoidanceX;
      outY = alpha * outY + (1 - alpha) * lastAvoidanceY;
      
      lastAvoidanceX = outX;
      lastAvoidanceY = outY;
      lastAvoidTime = now;
    }
    
    // Limita saída
    if (outX > 9) outX = 9;
    if (outX < -9) outX = -9;
    if (outY > 9) outY = 9;
    if (outY < -9) outY = -9;
  }
  
  // Comando de parada de emergência
  void emergencyStop() {
    PWM(0, 0);
    Serial.println("🚨 Parada de emergência por risco de colisão");
  }

  
  // Publica status de evitação para debug
  void publishAvoidanceStatus() {
    String nearestRobot;
    float nearestDist;
    int risk = checkCollisionRisk(nearestDist, nearestRobot);
    
    StaticJsonDocument<256> doc;
    doc["robot_id"] = robotId;
    doc["risk_level"] = risk;
    doc["nearest_robot"] = nearestRobot;
    doc["nearest_distance"] = nearestDist;
    doc["active_robots"] = otherRobots.size();
    
    char buffer[256];
    serializeJson(doc, buffer);
    publishMQTT("robot/avoidance", buffer);
  }
  
  int getActiveRobotsCount() {
    return otherRobots.size();
  }
  
  void printStatus() {
    Serial.printf("📊 Robôs próximos: %d\n", otherRobots.size());
    for (auto& pair : otherRobots) {
      OtherRobot& other = pair.second;
      Serial.printf("  - %s: dist=%.2fm, TTC=%.1fs\n", 
                    other.id, other.distance, other.timeToCollision);
    }
  }
};

// Instância global
CollisionAvoidance collisionAvoidance;

#endif
