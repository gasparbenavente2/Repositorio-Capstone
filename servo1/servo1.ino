#include <Servo.h>

Servo servo1;

int pos1 = 0;               // current angle
int stepDeg1 = 1;           // change per update (smaller = smoother)
unsigned long lastMove1 = 0;
const unsigned long stepMs1 = 200; // time between steps (smaller = faster)

void setup() {
  Serial.begin(9600);       // Monitor serie

  servo1.attach(9);
  delay(1000);

  servo1.writeMicroseconds(750);
}

void loop() {
  unsigned long now = millis();

  if (now - lastMove1 >= stepMs1) {
    lastMove1 = now;
    pos1 += stepDeg1;

    // reverse direction at endpoints
    if (pos1 >= 180 || pos1 <= 0) {
      stepDeg1 = -stepDeg1;
      pos1 += stepDeg1;
    }

    Serial.print(pos1);
    Serial.print('\n');
    servo1.write(pos1);
  }
}
