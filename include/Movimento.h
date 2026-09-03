#ifndef MOVIMENTO_H
#define MOVIMENTO_H

#include "Odometry.h"
float lerVelocRealEsquerda();
float lerVelocRealDireita();
//quarto
// Padrões de LEDs para cada movimento
//colocar random???
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

void atualizarOdometria(){
    static unsigned long tempoFinal = 0;
    unsigned long tempoAtual = millis();
    float deltaTempo = (tempoAtual - tempoFinal) / 1000.0; // (s))
    if (deltaTempo < 0.01) return; //evita divisao por zero

    int deltaPulsosDir = pulsosEncoderD - lastPulsosEncoderD;// Atualização de quantos pulsos desde a leitura mais nova 
    int deltaPulsosEsq = pulsosEncoderE - lastPulsosEncoderE; 

    float distRodaDir = deltaPulsosDir * DIST_POR_PULSO; //Distância percorrida pela roda (m)
    float distRodaEsq = deltaPulsosEsq * DIST_POR_PULSO;

    float velocRodaDir = distRodaDir / deltaTempo; //Velocidade da roda (m/s)
    float velocRodaEsq = distRodaEsq / deltaTempo;

    robotVx = (velocRodaDir + velocRodaEsq) / 2.0; //Velocidade linear do robô
    robotVtheta = (velocRodaDir - velocRodaEsq) / BASE_RODAS; //Velocidade angular do robô (rad/s)

    //Atualizar posição
    robotX += robotVx * cos(robotTheta) * deltaTempo;
    robotY += robotVx * sin(robotTheta) * deltaTempo;
    robotTheta += robotVtheta * deltaTempo;

    //Normalizar para [-pi, pi], evitar ângulos confusos e saber sempre paar onde está opontando
    //evita overflow, facilita comparação como os ângulos são cíclicos
    while(robotTheta > PI){
        robotTheta -= 2*PI;
    }
    while(robotTheta < -PI){
        robotTheta += 2*PI;
    }

    distTotalPercorrida += abs(robotVx) * deltaTempo;

    lastPulsosEncoderD = pulsosEncoderD;
    lastPulsosEncoderE = pulsosEncoderE;
    tempoFinal = tempoAtual;
}

void resetarOdometria(){
    robotX = 0.0;
    robotY = 0.0;
    robotTheta = 0.0;
    robotVx = 0.0;
    robotVy = 0.0;
    robotVtheta = 0.0;
    distTotalPercorrida = 0.0;

    pulsosEncoderD = 0;
    pulsosEncoderE = 0;
    lastPulsosEncoderD = 0;
    lastPulsosEncoderE = 0;

    Serial.println("🔄 Odometria resetada!");
}

void atualizarVelocidades(){
    velocRealEsquerda = lerVelocRealEsquerda();
    velocRealDireita = lerVelocRealDireita();
}

float lerVelocRealEsquerda(){
    static unsigned long tempoFinal = 0;
    static int pulsoFinal = 0;

    unsigned long tempoAtual = millis();
    unsigned long deltaTempo = tempoAtual - tempoFinal;
    if (deltaTempo < 10) return 0;

    int pulsoAtual = pulsosEncoderE;
    int deltaPulso = pulsoAtual - pulsoFinal;

    float velocidade = (deltaPulso*DIST_POR_PULSO) / (deltaTempo/1000.0);

    tempoFinal = tempoAtual;
    pulsoFinal = pulsoAtual;

    return velocRealEsquerda; // m/s
}

float lerVelocRealDireita(){
    static unsigned long tempoFinal = 0;
    static int pulsoFinal = 0;

    unsigned long tempoAtual = millis();
    unsigned long deltaTempo = tempoAtual - tempoFinal;
    if (deltaTempo < 10) return 0; //evita divisao por zero

    int pulsoAtual = pulsosEncoderD;
    int deltaPulso = pulsoAtual - pulsoFinal;

    float velocidade = (deltaPulso*DIST_POR_PULSO) / (deltaTempo/1000.0);

    tempoFinal = tempoAtual;
    pulsoFinal = pulsoAtual;

    return velocRealDireita; // m/s
}

float calcularPID(float erro){
    float erroProporcional = erro;

    erroIntegral += erro;
    if (erroIntegral > 5.0) erroIntegral = 5.0; //evitar overshoot
    if (erroIntegral < -5.0) erroIntegral = -5.0;

    erroDerivativo = erro - erroAnterior;
    erroAnterior = erro;

    float correcao_PID = (Kp*erroProporcional) + (Ki*erroIntegral) + (Kd*erroDerivativo);
    if (correcaoPID > correcaoMaxPID) {correcaoPID = correcaoMaxPID;}
    if (correcaoPID < -correcaoMaxPID) {correcaoPID = -correcaoMaxPID;}

    return correcao_PID;
}

void resetarPID(){
    erroAnterior = 0;
    erroIntegral = 0;
    correcaoPID = 0;
}

void PWM_PID(int direcaoX, int direcaoY) {
    atualizarOdometria();

    velocDesejada = direcaoX;
    rotacDesejada = direcaoY;
    
    int resultBrutoEsq = (direcaoX + direcaoY) * fatorCorrecaoEsquerda;
    int resultBrutoDir = (direcaoX - direcaoY) * fatorCorrecaoDireita;
    
    atualizarVelocidades();

    float erro = velocRealDireita - velocRealEsquerda;

    // Correção
    correcaoPID = calcularPID(erro);

    // Correção em linha reta
    int resultLiquidoEsq, resultLiquidoDir;
    if (direcaoX != 0 && direcaoY == 0){
        resultLiquidoEsq = resultBrutoEsq + correcaoPID;
        resultLiquidoDir = resultBrutoDir - correcaoPID;
    } else {
        resultLiquidoDir = resultBrutoDir;
        resultLiquidoEsq = resultBrutoEsq;
    }

    //Limita valores para o PWM
    if(resultLiquidoEsq > 9){ 
        resultLiquidoEsq = 9; 
    }
    if(resultLiquidoEsq < -9){ 
        resultLiquidoEsq = -9; 
    }
    if(resultLiquidoDir > 9){ 
        resultLiquidoDir = 9; 
    }
    if(resultLiquidoDir < -9){
        resultLiquidoDir = -9; 
    }

    // Converte para PWM
    int PWMEsq, PWMDir;
    
    if(resultLiquidoEsq == 0) {
        PWMEsq = 0;
    } else if(resultLiquidoEsq > 0) {
        PWMEsq = 120 + (resultLiquidoEsq * 15);
    }else {
        PWMEsq = -120 + (resultLiquidoEsq * 15);
    }

    if(resultLiquidoDir == 0) {
        PWMDir = 0;
    } else if(resultLiquidoDir > 0) {
        PWMDir = 120 + (resultLiquidoDir * 15);
    } else {
        PWMDir = -120 + (resultLiquidoDir * 15);
    }

    // Para os motores
    // ledcWrite(canal, valor) (0 até 15 no esp, 0 até 255)
    if(PWMEsq >= 0) {
        ledcWrite(1, PWMEsq);
        ledcWrite(0, 0);
    } else {
        ledcWrite(1, 0);
        ledcWrite(0, -PWMEsq);
    }
    if(PWMDir >= 0) {
        ledcWrite(3, PWMDir);
        ledcWrite(2, 0);
    } else {
        ledcWrite(3, 0);
        ledcWrite(2, -PWMDir);
    }

    // Controle de LEDs baseado no movimento
    if (direcaoX > 0 && direcaoY == 0) ledsFrente();
    else if (direcaoX < 0 && direcaoY == 0) ledsTras();
    else if (direcaoY < 0 && direcaoX == 0) ledsGiraEsquerda();
    else if (direcaoY > 0 && direcaoX == 0) ledsGiraDireita();
    else if (direcaoX == 0 && direcaoY == 0) ledsParado();
    
    else if (direcaoX > 0 && direcaoY > 0){ 
        ledsFrente(); digitalWrite(led[2], HIGH); 
        digitalWrite(led[5], HIGH); 
    }
    else if (direcaoX > 0 && direcaoY < 0){ 
        ledsFrente(); digitalWrite(led[0], HIGH); 
        digitalWrite(led[3], HIGH); 
    }
    else if (direcaoX < 0 && direcaoY > 0){
        ledsTras(); 
        digitalWrite(led[2], HIGH); 
        digitalWrite(led[5], HIGH); 
    }
    else if (direcaoX < 0 && direcaoY < 0){
        ledsTras(); 
        digitalWrite(led[0], HIGH); 
        digitalWrite(led[3], HIGH); 
    }
}

void para() {
    PWM_PID(0, 0);
    ledsParado();
    delay(50);
}

void SetupLeds() {
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    
    // Configuração dos canais   
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

//Movimentos precisos
void movFrentePreciso(float distMetros) {
    resetarPID();
    
    float posicaoInicial = robotX;
    float distanciaPercorrida = 0;
    int sinal = (distMetros > 0) ? 1 : -1; //1 para frente, -1 para trás
    float distanciaRestante = abs(distMetros);
        
    while(distanciaRestante > 0.005) {  // Precisão de 0.5cm
        PWM_PID(sinal * 9, 0);  
        delay(10);
        
        distanciaPercorrida = abs(robotX - posicaoInicial);
        distanciaRestante = abs(distMetros) - distanciaPercorrida;
        Serial.printf("🚶 Andando %.2f metros...\n", distMetros);
    }
    para();
    Serial.printf("✅ Andou %.2f metros\n", distMetros);
}

void girarGraus(float anguloGraus) {
    float anguloRad = anguloGraus * PI / 180.0; //graus para radianos
    resetarPID();
    
    float thetaInicial = robotTheta;
    float thetaPercorrido = 0;
    int sinal = (anguloGraus > 0) ? 1 : -1; //1 para direita, -1 para esquerda
    Serial.printf("🔄 Girando %.1f graus...\n", anguloGraus);
    
    while(abs(thetaPercorrido) < abs(anguloRad) - 0.01) {
        PWM_PID(0, sinal * 4);
        delay(10);
        
        thetaPercorrido = robotTheta - thetaInicial;
        while(thetaPercorrido > PI) thetaPercorrido -= 2 * PI;
        while(thetaPercorrido < -PI) thetaPercorrido += 2 * PI;
    }  
    para();
    Serial.printf("✅ Girou %.1f graus\n", anguloGraus);
}

//Movimentos sem PID direto
void movFrente(int timer) {
    PWM_PID(9, 0);
    delay(timer);
    para();
}

void movTras(int timer) {
    PWM_PID(-9, 0);
    delay(timer);
    para();
}

void giroDir(int timer) {
    PWM_PID(0, 4);
    delay(timer);
    para();
}

void giroEsq(int timer) {
    PWM_PID(0, -4);
    delay(timer);
    para();
}

void curvaNoroesteEF(int timer) {
    PWM_PID(5, -5);
    delay(timer);
    para();
}

void curvaSudoesteET(int timer) {
    PWM_PID(-5, 5);
    delay(timer);
    para();
}

void curvaNordesteDF(int timer) {
    PWM_PID(5, 5);
    delay(timer);
    para();
}

void curvaSudesteDT(int timer) {
    PWM_PID(-5, -5);
    delay(timer);
    para();
}

void desenharTrianguloEquilatero(float lado=0.5){
    Serial.println("🔺 Desenhando triângulo equilátero...");

    resetarOdometria();

    for(int i=0; i < 3; i++){
        movFrentePreciso(lado);
        girarGraus(120);
    }
    Serial.println("✅ Triângulo feito!");
}

void desenharQuadrado(float lado = 0.5){
    Serial.println("🔲 Desenhando quadrado...");

    resetarOdometria();

    for(int i=0; i < 4; i++){
        movFrentePreciso(lado);
        girarGraus(90);
    }
    Serial.println("✅ Quadrado feito!"); 
}

void desenharCoracao(float tamanho = 0.5) {
    Serial.println("❤️ Desenhando coração...");
    resetarOdometria();
    
    float lado = tamanho / 2;
    
    // Metade esquerda do coração
    movFrentePreciso(lado);
    girarGraus(45);
    movFrentePreciso(lado);
    girarGraus(45);
    movFrentePreciso(lado);
    girarGraus(45);
    movFrentePreciso(lado);
    
    // Curva inferior (aproximada)
    girarGraus(90);
    movFrentePreciso(lado * 0.7);
    girarGraus(45);
    movFrentePreciso(lado * 0.7);
    girarGraus(45);
    movFrentePreciso(lado * 0.7);
    
    // Metade direita do coração (simétrica)
    girarGraus(45);
    movFrentePreciso(lado * 0.7);
    girarGraus(45);
    movFrentePreciso(lado * 0.7);
    girarGraus(90);
    movFrentePreciso(lado);
    girarGraus(45);
    movFrentePreciso(lado);
    girarGraus(45);
    movFrentePreciso(lado);
    girarGraus(45);
    movFrentePreciso(lado);
    
    Serial.println("✅ Coração completo!");
}

#endif