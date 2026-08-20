#ifndef MULTI_ROBOT_COORD_H
#define MULTI_ROBOT_COORD_H

#include "Configs.h"
#include "Odometry.h"
#include "CollisionAvoidance.h"
#include "Localization.h"

// Modos de coordenação
enum CoordinationMode {
  COORD_FREE = 0,        // Livre (apenas evita colisão)
  COORD_LEADER_FOLLOWER, // Líder-seguidor
  COORD_FORMATION,       // Formação
  COORD_AREA_COVERAGE    // Cobertura de área
};

// Papéis do robô
enum RobotRole {
  ROLE_FREE = 0,
  ROLE_LEADER,
  ROLE_FOLLOWER,
  ROLE_EXPLORER
};

class MultiRobotCoordinator {
private:
  CoordinationMode mode = COORD_FREE;
  RobotRole role = ROLE_FREE;
  
  // Para modo líder-seguidor
  char leaderId[16] = "";
  float followDistance = 0.5f;   // Distância desejada do líder (metros)
  float followAngle = 0.0f;      // Ângulo desejado em relação ao líder
  
  // Para formação
  float formationOffsetX = 0;
  float formationOffsetY = 0;
  
  // Para cobertura de área
  float gridSize = 1.0f;         // Tamanho da célula (metros)
  bool* visitedCells = nullptr;
  int gridWidth = 0, gridHeight = 0;
  
  // Referência ao líder (cache)
  OtherRobot leaderInfo;
  bool leaderInfoValid = false;
  
public:
  void setup(int gridW = 5, int gridH = 5) {
    if (mode == COORD_AREA_COVERAGE) {
      gridWidth = gridW;
      gridHeight = gridH;
      visitedCells = new bool[gridW * gridH];
      for (int i = 0; i < gridW * gridH; i++) {
        visitedCells[i] = false;
      }
    }
  }
  
  void setMode(CoordinationMode newMode) {
    mode = newMode;
    Serial.printf("🔄 Modo de coordenação alterado para: %d\n", mode);
    publishModeStatus();
  }
  
  void setRole(RobotRole newRole) {
    role = newRole;
    Serial.printf("🎭 Papel alterado para: %d\n", role);
    publishRoleStatus();
  }
  
  void setLeader(const char* id) {
    strcpy(leaderId, id);
    role = ROLE_FOLLOWER;
    Serial.printf("👑 Líder definido: %s\n", leaderId);
  }
  
  void setFormationOffset(float offsetX, float offsetY) {
    formationOffsetX = offsetX;
    formationOffsetY = offsetY;
  }
  
  void updateLeaderInfo() {
    // Busca informações do líder no mapa de outros robôs
    // (implementação depende de como os dados são acessados)
    leaderInfoValid = false;
    
    // Nota: Seria necessário acesso ao mapa de otherRobots do CollisionAvoidance
    // Por simplicidade, deixamos como esboço
  }
  
  // Calcula o comando desejado baseado no modo de coordenação
  void getCoordinatedCommand(int& targetX, int& targetY, int& targetTheta) {
    targetX = 0;
    targetY = 0;
    targetTheta = 0;
    
    switch (mode) {
      case COORD_LEADER_FOLLOWER:
        if (leaderInfoValid && role == ROLE_FOLLOWER) {
          // Calcular posição desejada em relação ao líder
          float desiredX = leaderInfo.x - followDistance * cos(leaderInfo.theta + followAngle);
          float desiredY = leaderInfo.y - followDistance * sin(leaderInfo.theta + followAngle);
          
          // Erro de posição
          float errorX = desiredX - robotX;
          float errorY = desiredY - robotY;
          
          // Controle proporcional
          targetX = constrain((int)(errorX * 10), -9, 9);
          targetY = constrain((int)(errorY * 10), -9, 9);
        }
        break;
        
      case COORD_FORMATION:
        // Mantém formação relativa a um ponto de referência
        // Por enquanto, apenas mantém posição
        targetX = 0;
        targetY = 0;
        break;
        
      case COORD_AREA_COVERAGE:
        // Navegação para células não visitadas
        getCoverageCommand(targetX, targetY);
        break;
        
      default:
        break;
    }
  }
  
  void getCoverageCommand(int& velX, int& velY) {
    // Algoritmo simples de varredura
    static int currentCellX = 0, currentCellY = 0;
    int cellX = floor(robotX / gridSize);
    int cellY = floor(robotY / gridSize);
    
    if (cellX >= 0 && cellX < gridWidth && cellY >= 0 && cellY < gridHeight) {
      visitedCells[cellY * gridWidth + cellX] = true;
    }
    
    // Encontra célula não visitada mais próxima
    int nearestUnvisitedX = -1, nearestUnvisitedY = -1;
    float minDist = 999;
    
    for (int y = 0; y < gridHeight; y++) {
      for (int x = 0; x < gridWidth; x++) {
        if (!visitedCells[y * gridWidth + x]) {
          float dist = sqrt(pow(x * gridSize - robotX, 2) + 
                           pow(y * gridSize - robotY, 2));
          if (dist < minDist) {
            minDist = dist;
            nearestUnvisitedX = x;
            nearestUnvisitedY = y;
          }
        }
      }
    }
    
    if (nearestUnvisitedX >= 0) {
      // Navega em direção à célula
      float dx = nearestUnvisitedX * gridSize - robotX;
      float dy = nearestUnvisitedY * gridSize - robotY;
      
      velX = constrain((int)(dx * 5), -9, 9);
      velY = constrain((int)(dy * 5), -9, 9);
    }
  }
  
  void publishModeStatus() {
    StaticJsonDocument<256> doc;
    doc["robot_id"] = robotId;
    doc["mode"] = mode;
    doc["role"] = role;
    doc["active"] = true;
    
    char buffer[256];
    serializeJson(doc, buffer);
    publishMQTT("robot/coord_mode", buffer);
  }
  
  void publishRoleStatus() {
    StaticJsonDocument<256> doc;
    doc["robot_id"] = robotId;
    doc["role"] = role;
    if (strlen(leaderId) > 0) {
      doc["leader"] = leaderId;
    }
    
    char buffer[256];
    serializeJson(doc, buffer);
    publishMQTT("robot/role", buffer);
  }
  
  void printStatus() {
    Serial.printf("📡 Coordenação: modo=%d, papel=%d\n", mode, role);
    if (role == ROLE_FOLLOWER && strlen(leaderId) > 0) {
      Serial.printf("  Seguindo líder: %s\n", leaderId);
    }
  }
};

// Instância global
MultiRobotCoordinator multiRobotCoord;

#endif