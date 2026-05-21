#ifndef MOVIMENTO_H
#define MOVIMENTO_H

void PWM(int dirX,int dirY){
  // Aplica correção automática apenas no eixo X (movimento frente/trás)
  int Resq = (dirX + dirY) * fatorCorrecaoEsquerda;
  int Rdir = (dirX - dirY) * fatorCorrecaoDireita;
  
  if(Resq>9) {Resq = 9;}
  if(Resq<-9) {Resq = -9;}
  if(Rdir>9) {Rdir = 9;}
  if(Rdir<-9) {Rdir = -9;}
  
  int PWMe;
  int PWMd;
  
  if(Resq == 0) {PWMe = 0;}
  if(Resq > 0) {PWMe = 120 + Resq*15;}
  if(Resq < 0) {PWMe = -120 + Resq*15;}

  if(Rdir == 0) {PWMd = 0;}
  if(Rdir > 0) {PWMd = 120 + Rdir*15;}
  if(Rdir < 0) {PWMd = -120 + Rdir*15;}

  if(PWMe>=0) {
    ledcWrite(1, PWMe);
    ledcWrite(0, 0);
  } else {
    ledcWrite(1, 0);
    ledcWrite(0, -PWMe);
  }
  if(PWMd>=0) {
    ledcWrite(3, PWMd);
    ledcWrite(2, 0);
  } else {
    ledcWrite(3, 0);
    ledcWrite(2, -PWMd);
  }
}

//"para()" serve para que o carrinho pare qualquer movimento ->
void para(){
  PWM(0,0);  
  delay(50);
}

// Função para calibrar automaticamente
void calibrarMotores() {
    if (!calibracaoAtiva) {
        calibracaoAtiva = true;
        tempoCalibracao = millis();
        resetEncoders();
        Serial.println("[CALIBRACAO] Iniciando calibração automática...");
        
        // Move para frente por 3 segundos para calibrar
        unsigned long startTime = millis();
        while (millis() - startTime < 3000) {
            // PWM direto sem correção para calibração
            int Resq = 9;
            int Rdir = 9;
            
            int PWMe = 120 + Resq*15;
            int PWMd = 120 + Rdir*15;
            
            ledcWrite(1, PWMe);
            ledcWrite(0, 0);
            ledcWrite(3, PWMd);
            ledcWrite(2, 0);
            
            delay(10);
            
            // Verifica se atingiu pulsos suficientes
            if (pulsosEncoderE >= pulsosAlvoCalibracao && pulsosEncoderD >= pulsosAlvoCalibracao) {
                break;
            }
        }
        para();
        
        // Calcula a correção
        if (pulsosEncoderE > 0 && pulsosEncoderD > 0) {
            float razao = (float)pulsosEncoderD / pulsosEncoderE;
            fatorCorrecaoEsquerda = 1.0 * razao;
            fatorCorrecaoDireita = 1.0;
            
            Serial.print("[CALIBRACAO] Pulsos E:");
            Serial.print(pulsosEncoderE);
            Serial.print(" D:");
            Serial.print(pulsosEncoderD);
            Serial.print(" | Razão: ");
            Serial.print(razao);
            Serial.print(" | Correção Esquerda: ");
            Serial.println(fatorCorrecaoEsquerda);
        }
        
        calibracaoAtiva = false;
        resetEncoders();
    }
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

  for(int i = 0; i<8;i++){
    pinMode(led[i],OUTPUT);
  };
}

void ligaLeds(){
  iled = 0;
  while(iled<8) {
    digitalWrite(led[iled],HIGH);
    iled++;
  }
}

void desligaLeds(){
  iled = 0;
  while(iled<8) {
    digitalWrite(led[iled],LOW);
    iled++;
  }
}

void acendeLeds(int a, int b, int c, int d, int e, int f, int g, int h){
  if (a == 1) {
    digitalWrite(led[0],HIGH);
  } else {
    digitalWrite(led[0],LOW);
  }
  if (b == 1) {
    digitalWrite(led[1],HIGH);
  } else {
    digitalWrite(led[1],LOW);
  }
  if (c == 1) {
    digitalWrite(led[2],HIGH);
  } else {
    digitalWrite(led[2],LOW);
  }
  if (d == 1) {
    digitalWrite(led[3],HIGH);
  } else {
    digitalWrite(led[3],LOW);
  }
  if (e == 1) {
    digitalWrite(led[4],HIGH);
  } else {
    digitalWrite(led[4],LOW);
  }
  if (f == 1) {
    digitalWrite(led[5],HIGH);
  } else {
    digitalWrite(led[5],LOW);
  }
  if (g == 1) {
    digitalWrite(led[6],HIGH);
  } else {
    digitalWrite(led[6],LOW);
  }
  if (h == 1) {
    digitalWrite(led[7],HIGH);
  } else {
    digitalWrite(led[7],LOW);
  }
  
}

//"movF()" serve para que o carrinho realize o movimento frontal ->
void movF(int timer){
  // Calibra na primeira execução
  static bool primeiraVez = true;
  if (primeiraVez) {
      calibrarMotores();
      primeiraVez = false;
  }
  
  PWM(9,0);
  delay(50); 
  delay(timer);
}

//"movT()" serve para que o carrinho realize o movimento trazeiro ->
void movT(int timer){
  PWM(-9,0);
  delay(50);
  delay(timer);
}

//"girD()" serve para que o carrinho realize o movimento circular horário em seu próprio eixo ->
void girD(int timer) {
  PWM(0,4);
  delay(50);
  delay(timer);
}

//"girE()" serve para que o carrinho realize o movimento circular anti-horário em seu próprio eixo ->
void girE(int timer) {
  PWM(0,-4);
  delay(50);
  delay(timer);
}

void giroe(int timer){
  Serial.println(timer);
}

//"rotE_F()" serve para que o carrinho realize o movimento circular anti-horário no eixo da roda esquerda ->
void rotE_F(int timer) {
  PWM(+5,-5);
  delay(50);
  delay(timer);
}

//"rotE_T()" serve para que o carrinho realize o movimento circular horário no eixo da roda esquerda ->
void rotE_T(int timer) {
  PWM(-5,+5);
  delay(50);
  delay(timer);
}

//"rotD_F()" serve para que o carrinho realize o movimento circular anti-horário no eixo da roda direita ->
void rotD_F(int timer) {
  PWM(+5,+5);
  delay(50);
  delay(timer);
}

//"rotD_T()" serve para que o carrinho realize o movimento circular horário no eixo da roda direita ->
void rotD_T(int timer) {
  PWM(-5,-5);
  delay(50);
  delay(timer);
}

#endif