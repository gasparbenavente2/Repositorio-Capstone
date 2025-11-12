#include <Servo.h>

// ----- Servo -----
int pos1 = 0;               // current angle
int stepDeg1 = 1;           // change per update (smaller = smoother)
unsigned long lastMove1 = 0;
const unsigned long stepMs1 = 20; // time between steps (smaller = faster)
int pos2 = 60;               // 60 abajo
int stepDeg2 = 1;           // change per update (smaller = smoother)
unsigned long lastMove2 = 0;
const unsigned long stepMs2 = 150; // time between steps (smaller = faster)
Servo servo1; //servo object
Servo servo2;

// BAUD del Sabertooth debe coincidir
const int SPEED = 50;
const unsigned long MOVE_MS = 3000;
const unsigned long PAUSE_MS = 2000;

// Pines encoder
const uint8_t ENC1_A = 21;   // Motor 1 A (interrupción)
const uint8_t ENC1_B = 2;    // Motor 1 B
const uint8_t ENC2_A = 20;   // Motor 2 A (interrupción)
const uint8_t ENC2_B = 3;    // Motor 2 B

// ----- Limit switches -----
const uint8_t LIM1_PIN = 24;   // limit for Motor 1
const uint8_t LIM2_PIN = 25;   // limit for Motor 2

// ----- Encoders -----
volatile long enc1 = 0, enc2 = 0;
volatile uint8_t e1_prev = 0, e2_prev = 0;
bool lim1WasPressed = false;

// delta lookup for quadrature (prev rows 00..11, curr cols 00..11)
inline int8_t qdelta(uint8_t p, uint8_t c) {
  static const int8_t t[4][4] = {
    { 0, -1, +1,  0},
    { +1, 0,  0, -1},
    { -1, 0,  0, +1},
    { 0, +1, -1,  0}
  };
  return t[p & 3][c & 3];
}

void isr_e1() {
  uint8_t a = digitalRead(ENC1_A);
  uint8_t b = digitalRead(ENC1_B);
  uint8_t s = (a << 1) | b;
  enc1 += qdelta(e1_prev, s);
  e1_prev = s;
}
void isr_e2() {
  uint8_t a = digitalRead(ENC2_A);
  uint8_t b = digitalRead(ENC2_B);
  uint8_t s = (a << 1) | b;
  enc2 += qdelta(e2_prev, s);
  e2_prev = s;
}

void setup() {
  // SERVOS
  servo1.attach(9);
  servo1.write(pos1);
  // servo2.attach(8);
  // servo2.write(60);
  // delay(2000);
  // servo2.write(75);
  // delay(2000);
  // servo2.write(60);


  Serial1.begin(9600);    // Sabertooth
  Serial.begin(9600);       // Monitor serie
  delay(100);

  Serial1.println("M1:0 M2:0");
  delay(1000);
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);

    // init previous states
  e1_prev = (digitalRead(ENC1_A) << 1) | digitalRead(ENC1_B);
  e2_prev = (digitalRead(ENC2_A) << 1) | digitalRead(ENC2_B);

  // 4x decoding: interrupt on both lines of each encoder
  attachInterrupt(digitalPinToInterrupt(ENC1_A), isr_e1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), isr_e1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), isr_e2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), isr_e2, CHANGE);

  noInterrupts();
  enc1 = enc2 = 0;
  interrupts();

  pinMode(LIM1_PIN, INPUT_PULLUP);   // NC to GND → reads LOW normally
  pinMode(LIM2_PIN, INPUT_PULLUP);
  lim1WasPressed = (digitalRead(LIM1_PIN) == LOW);

  Serial1.println("M1:-120");    // bajada
  delay(50000);
  Serial1.println("M1:0");

  // Serial1.println("M1:0");    // pausa
  // delay(3000);

  // Serial1.println("M2:300");   //subida
  // delay(3500);
  // Serial1.println("M2:0");

  // Serial1.println("M2:0");
  // delay(2000);

  // Serial1.println("M2:-150");   //bajada
  // delay(3500);
  // Serial1.println("M2:0");
  // delay(2000);

  // Serial1.println("M1:300");    // subida
  // delay(2000);
  // Serial1.println("M1:0");
}

void loop() {
  bool lim1Pressed = (digitalRead(LIM1_PIN) == LOW);
  // long enc1Copy, enc2Copy;

  // if (lim1Pressed && !lim1WasPressed) {
  //   // Reset encoder 1 count the moment the limit switch engages
  //   noInterrupts();
  //   enc1 = 0;
  //   e1_prev = (digitalRead(ENC1_A) << 1) | digitalRead(ENC1_B);
  //   interrupts();
  // }
  // lim1WasPressed = lim1Pressed;

  // noInterrupts();
  // enc1Copy = enc1;
  // enc2Copy = enc2;
  // interrupts();

  // Serial.print("Encoder1: "); Serial.print(enc1Copy);
  // Serial.print(" | Encoder2: "); Serial.println(enc2Copy);

  // if (enc1Copy < 1920 && !lim1Pressed) {
  //   Serial1.println("M1:100");
  //   Serial1.println("M2:100");
  // } else {
  //   Serial1.println("M1:0");
  //   Serial1.println("M2:0");
  // }

  //Sweep 0-180
  // unsigned long now = millis();

  // if (now - lastMove1 >= stepMs1) {
  //   lastMove1 = now;
  //   pos1 += stepDeg1;

  //   // reverse direction at endpoints
  //   if (pos1 >= 180 || pos1 <= 0) {
  //     stepDeg1 = -stepDeg1;
  //     pos1 += stepDeg1;
  //   }

  //   servo1.write(pos1);
  // }

  // if (now - lastMove2 >= stepMs2) {
  //   lastMove2 = now;
  //   pos2 += stepDeg2;

  //   if (pos2 >= 75 || pos2 <= 60) {
  //     stepDeg2 = -stepDeg2;
  //     pos2 += stepDeg2;
  //   }

  //   //servo2.write(pos2);
  // }

}
