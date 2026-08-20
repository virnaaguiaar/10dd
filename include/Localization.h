#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include "Configs.h"
#include "Odometry.h"
#include <Wire.h>
#include <math.h>

// Filtro de Kalman simples para fusão de sensores
class KalmanFilter {
private:
  float Q_angle;    // Ruído do processo (ângulo)
  float Q_bias;     // Ruído do processo (bias do giroscópio)
  float R_measure;  // Ruído da medição
  
  float angle;      // Ângulo estimado
  float bias;       // Bias estimado
  float P[2][2];    // Matriz de covariância do erro
  
public:
  KalmanFilter() {
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;
    
    angle = 0;
    bias = 0;
    P[0][0] = 0;
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 0;
  }
  
  float getAngle(float newAngle, float newRate, float dt) {
    // Predição
    float rate = newRate - bias;
    angle += dt * rate;
    
    // Atualiza matriz de covariância do erro
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    
    // Atualização da medição
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    
    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;
    
    // Atualiza covariância
    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];
    
    return angle;
  }
  
  void reset() {
    angle = 0;
    bias = 0;
    P[0][0] = 0;
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 0;
  }
};

// Estrutura para representar pose com incerteza
struct PoseWithCovariance {
  float x, y, theta;
  float cov_xx, cov_yy, cov_tt;  // Variâncias
  float timestamp;
};

class LocalizationSystem {
private:
  KalmanFilter kalmanX, kalmanY, kalmanTheta;
  
  // Parâmetros da MPU6050 (valores típicos)
  float gyroScale = 131.0f;  // LSB por grau/seg para ±250°/s
  float accelScale = 16384.0f; // LSB por g para ±2g
  
  // Últimos valores do giroscópio
  float lastGyroX = 0, lastGyroY = 0, lastGyroZ = 0;
  unsigned long lastSensorTime = 0;
  
  // Fusão de dados
  float fusedX, fusedY, fusedTheta;
  
public:
  void setup() {
    // Inicializa I2C para MPU6050
    Wire.begin(sda_gyro, slc_gyro);
    Wire.setClock(400000);
    
    // Inicializa MPU6050
    initMPU6050();
    
    kalmanX.reset();
    kalmanY.reset();
    kalmanTheta.reset();
    
    lastSensorTime = micros();
  }
  
  void initMPU6050() {
    // Wake up MPU6050
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    
    // Configurar aceleração para ±2g
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission();
    
    // Configurar giroscópio para ±250°/s
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission();
    
    delay(100);
  }
  
  void readMPU6050(int16_t* ax, int16_t* ay, int16_t* az, 
                   int16_t* gx, int16_t* gy, int16_t* gz) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);  // Starting register for Accel data
    Wire.endTransmission(false);
    Wire.requestFrom(static_cast<uint8_t>(0x68), static_cast<size_t>(14), true);
    
    *ax = Wire.read() << 8 | Wire.read();
    *ay = Wire.read() << 8 | Wire.read();
    *az = Wire.read() << 8 | Wire.read();
    *gx = Wire.read() << 8 | Wire.read();
    *gy = Wire.read() << 8 | Wire.read();
    *gz = Wire.read() << 8 | Wire.read();
  }
  
  // Calcula ângulo de inclinação a partir do acelerômetro
  float getAccelAngle(int16_t ax, int16_t ay, int16_t az) {
    float ax_f = ax / accelScale;
    float ay_f = ay / accelScale;
    float az_f = az / accelScale;
    
    float pitch = atan2(-ax_f, sqrt(ay_f*ay_f + az_f*az_f)) * 180 / PI;
    float roll = atan2(ay_f, az_f) * 180 / PI;
    
    // Para movimento planar, usamos roll como ângulo de orientação
    return roll;
  }
  
  void updateFusion() {
    unsigned long now = micros();
    float dt = (now - lastSensorTime) / 1000000.0f;
    if (dt > 0.1) dt = 0.01;
    
    int16_t ax, ay, az, gx, gy, gz;
    readMPU6050(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Converte giroscópio para graus/segundo
    float gyroZ = gz / gyroScale;
    
    // Calcula ângulo do acelerômetro
    float accelAngle = getAccelAngle(ax, ay, az);
    
    // Filtro de Kalman para o ângulo
    fusedTheta = kalmanTheta.getAngle(accelAngle, gyroZ, dt);
    
    // Para posição X e Y, usamos apenas odometria com correção do giroscópio
    // Mas podemos usar aceleração para detecção de colisão
    float accelX = ax / accelScale;
    float accelY = ay / accelScale;
    
    // Detecta impacto (colisão) por aceleração
    if (fabs(accelX) > 2.0f || fabs(accelY) > 2.0f) {
      Serial.println("💥 COLISÃO DETECTADA PELO ACELERÔMETRO!");
      // Podemos ajustar a odometria aqui
    }
    
    lastSensorTime = now;
    lastGyroZ = gyroZ;
  }
  
  // Corrige a posição usando o ângulo do giroscópio
  void correctOdometryWithGyro() {
    updateFusion();
    
    // Aplica correção no ângulo do robô usando o giroscópio
    float angleError = fusedTheta - robotTheta;
    
    // Correção suave (filtro complementar)
    float alpha = 0.98f;  // Confiança no giroscópio
    robotTheta = alpha * fusedTheta + (1 - alpha) * robotTheta;
    
    // Normaliza o ângulo
    while (robotTheta > PI) robotTheta -= 2 * PI;
    while (robotTheta < -PI) robotTheta += 2 * PI;
  }
  
  // Publica pose com covariância para outros robôs
  void publishPoseWithCovariance() {
    StaticJsonDocument<512> doc;
    doc["robot_id"] = robotId;
    doc["x"] = robotX;
    doc["y"] = robotY;
    doc["theta"] = robotTheta;
    doc["vx"] = robotVx;
    doc["vy"] = robotVy;
    doc["vtheta"] = robotVtheta;
    
    // Adiciona covariância (incerteza da localização)
    doc["cov_xx"] = 0.01f;  // Incerteza típica de odometria
    doc["cov_yy"] = 0.01f;
    doc["cov_tt"] = 0.05f;
    
    doc["timestamp"] = millis() / 1000.0;
    
    char buffer[512];
    serializeJson(doc, buffer);
    
    String topic = String("robot/pose/") + robotId;
    publishMQTT(topic.c_str(), buffer);
  }
};

// Instância global
LocalizationSystem localization;

#endif
