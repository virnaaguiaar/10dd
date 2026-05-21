#ifndef MP3_H
#define MP3_H

extern SoftwareSerial softwareSerial;
extern DFRobotDFPlayerMini player;
//SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);

// Create the Player object
DFRobotDFPlayerMini player;

void Mp3Setup() {

  softwareSerial.begin(9600);

  if (player.begin(softwareSerial)) {
   Serial.println("Mp3 Online");

    // Set volume to maximum (0 to 30).
    player.volume(25);
    // Play the first MP3 file on the SD card
    //player.play(1);
  } else {
    Serial.println("Connecting to DFPlayer Mini failed!");
  }
}

#endif