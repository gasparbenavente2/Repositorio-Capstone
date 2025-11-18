#include <Servo.h>

Servo servo2;

void setup() {
  servo2.attach(9);
  servo2.write(60);
  delay(2000);
  servo2.write(75);
  delay(2000);
  servo2.write(60);
}

void loop() {

}
