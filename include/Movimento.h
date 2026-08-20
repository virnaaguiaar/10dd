#ifndef MOVIMENTO_H
#define MOVIMENTO_H

#include "Odometry.h"

// Padrões de LEDs para cada movimento
void ledsFrente() {
    digitalWrite(led[0], HIGH);
    digitalWrite(led[1], HIGH);
    digitalWrite(led[2], HIGH);
    digitalWrite(led[3], LOW);
    digitalWrite(led[4], LOW);
    digitalWrite(led[5], LOW);
    digitalWrite(led[6], LOW);
    digitalWrite(led[7], LOW);
}

void ledsTras() {
    digitalWrite(led[5], HIGH);
    digitalWrite(led[6], HIGH);
    digitalWrite(led[7], HIGH);
    digitalWrite(led[0], LOW);
    digitalWrite(led[1], LOW);
    digitalWrite(led[2], LOW);
    digitalWrite(led[3], LOW);
    digitalWrite(led[4], LOW);
}

void ledsGiraEsquerda() {
    digitalWrite(led[0], HIGH);
    digitalWrite(led[3], HIGH);
    digitalWrite(led[6], HIGH);
    digitalWrite(led[1], LOW);
    digitalWrite(led[2], LOW);
    digitalWrite(led[4], LOW);
    digitalWrite(led[5], LOW);
    digitalWrite(led[7], LOW);
}

void ledsGiraDireita() {
    digitalWrite(led[2], HIGH);
    digitalWrite(led[5], HIGH);
    digitalWrite(led[7], HIGH);
    digitalWrite(led[0], LOW);
    digitalWrite(led[1], LOW);
    digitalWrite(led[3], LOW);
    digitalWrite(led[4], LOW);
    digitalWrite(led[6], LOW);
}

void ledsParado() {
    for(int i = 0; i < 8; i++) {
        digitalWrite(led[i], LOW);
    }
}

void PWM(int dirX, int dirY) {
    int Resq = (dirX + dirY) * fatorCorrecaoEsquerda;
    int Rdir = (dirX - dirY) * fatorCorrecaoDireita;
    
    if(Resq > 9) { Resq = 9; }
    if(Resq < -9) { Resq = -9; }
    if(Rdir > 9) { Rdir = 9; }
    if(Rdir < -9) { Rdir = -9; }
    
    // Controle de LEDs baseado no movimento
    if (dirX > 0 && dirY == 0) ledsFrente();
    else if (dirX < 0 && dirY == 0) ledsTras();
    else if (dirY < 0 && dirX == 0) ledsGiraEsquerda();
    else if (dirY > 0 && dirX == 0) ledsGiraDireita();
    else if (dirX == 0 && dirY == 0) ledsParado();
    else if (dirX > 0 && dirY > 0) { ledsFrente(); digitalWrite(led[2], HIGH); digitalWrite(led[5], HIGH); }
    else if (dirX > 0 && dirY < 0) { ledsFrente(); digitalWrite(led[0], HIGH); digitalWrite(led[3], HIGH); }
    else if (dirX < 0 && dirY > 0) { ledsTras(); digitalWrite(led[2], HIGH); digitalWrite(led[5], HIGH); }
    else if (dirX < 0 && dirY < 0) { ledsTras(); digitalWrite(led[0], HIGH); digitalWrite(led[3], HIGH); }
    
    int PWMe, PWMd;
    
    if(Resq == 0) PWMe = 0;
    else if(Resq > 0) PWMe = 120 + Resq * 15;
    else PWMe = -120 + Resq * 15;
    
    if(Rdir == 0) PWMd = 0;
    else if(Rdir > 0) PWMd = 120 + Rdir * 15;
    else PWMd = -120 + Rdir * 15;
    
    if(PWMe >= 0) {
        ledcWrite(1, PWMe);
        ledcWrite(0, 0);
    } else {
        ledcWrite(1, 0);
        ledcWrite(0, -PWMe);
    }
    if(PWMd >= 0) {
        ledcWrite(3, PWMd);
        ledcWrite(2, 0);
    } else {
        ledcWrite(3, 0);
        ledcWrite(2, -PWMd);
    }
}

void para() {
    PWM(0, 0);
    ledsParado();
    delay(50);
}

void SetupLeds() {
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    
    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(2, PWM_FREQ, 8);
    ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(motor1Pin1, 0);
    ledcAttachPin(motor1Pin2, 1);
    ledcAttachPin(motor2Pin1, 2);
    ledcAttachPin(motor2Pin2, 3);
    
    for(int i = 0; i < 8; i++) {
        pinMode(led[i], OUTPUT);
    }
    ledsParado();
}

void ligaLeds() {
    for(int i = 0; i < 8; i++) digitalWrite(led[i], HIGH);
}

void desligaLeds() {
    for(int i = 0; i < 8; i++) digitalWrite(led[i], LOW);
}

void movF(int timer) {
    PWM(9, 0);
    delay(timer);
    para();
}

void movT(int timer) {
    PWM(-9, 0);
    delay(timer);
    para();
}

void girD(int timer) {
    PWM(0, 4);
    delay(timer);
    para();
}

void girE(int timer) {
    PWM(0, -4);
    delay(timer);
    para();
}

void rotE_F(int timer) {
    PWM(5, -5);
    delay(timer);
    para();
}

void rotE_T(int timer) {
    PWM(-5, 5);
    delay(timer);
    para();
}

void rotD_F(int timer) {
    PWM(5, 5);
    delay(timer);
    para();
}

void rotD_T(int timer) {
    PWM(-5, -5);
    delay(timer);
    para();
}

#endif