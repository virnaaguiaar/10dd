#ifndef MP3_H
#define MP3_H

#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

// Declaração externa (definida no main_h.ino)
extern SoftwareSerial softwareSerial;
extern DFRobotDFPlayerMini player;

void Mp3Setup() {
  softwareSerial.begin(9600);
  delay(100);
  
  if (player.begin(softwareSerial)) {
    Serial.println("✅ Mp3 Online");
    player.volume(25);     // Volume 0-30
    player.stop();         // Garante que não começa tocando
  } else {
    Serial.println("❌ Conexão com DFPlayer Mini falhou!");
  }
}

// Função auxiliar para tocar música
void tocarMusica(int id) {
  if (id >= 1 && id <= 4) {  // Músicas 1-4
    player.play(id);
    Serial.printf("🎵 Tocando música %d\n", id);
  } else {
    Serial.printf("⚠️ ID de música inválido: %d\n", id);
  }
}

// Função para parar música
void pararMusica() {
  player.stop();
  Serial.println("⏹️ Música parada");
}

#endif