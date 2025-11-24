#include <Servo.h>

// Servo 1
Servo servo1;
int s1pos1 = 130;               // current angle
int s1pos1limit = 80;

// Servo 2
Servo servo2;

// Definición de PINs.
// Encoders
#define encoder0PinA  21    // Yellow
#define encoder0PinB  2     // White
#define encoder1PinA  20    // Yellow
#define encoder1PinB  3     // White        // Green GND, Blue 5V

// Limit switches
#define LIM0_PIN 25         // Green
#define LIM1_PIN 24         // Green
bool lim0Pressed;
bool lim1Pressed;
bool lim0WasPressed;
bool lim1WasPressed;

// Sabertooth
// 5V a 18 Arduino

// Variables Tiempo
unsigned long time_ant = 0;
const int Period = 10000;                   // 10 ms = 100Hz
const float dt = Period *0.000001f;

// Variables de los Encoders y posicion
volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;
long newposition0;
long oldposition0 = 0;
long newposition1;
long oldposition1 = 0;
unsigned long newtime;
float vel0;
float vel1;
const long divM0 = 1920L * 28L;
const long divM1 = 1920L * 18L;
float revM0 = 0;
float revM1 = 0;
float degM0 = 0;
float degM1 = 0;

// Control macro
bool homing = false; // Cuando se prende el robot, se mueve lentamente hasta llegar a los enconders
float PID_control = true;
bool print_control = true; // imprimir señales en terminal
float setpoint0 = 0;     // eslabon 1
float setpoint1 = 0;  // eslabon 2

// homing
bool homing_m1 = true;  // No modificar
bool homing_m2 = true;  // No modificar
int homing_m1_speed = 320;
int homing_m2_speed = -60;


// Variables control PID motoes
int firstPID = true;
float pos_tol = 0.1; // Valor al que se considera alcanzado el angulo
int max_integrador = 100; 

// ------------------- M1 -----------------------
float control1;
float old_control1 = 0;
float error1;
float error_old1 = 0;
float error_old_old1 = 0;
int control1int = 0;

float KP_1 = 12;
float KI_1 = 4.5;
float KD_1 = 0.08;

// ------------------- M2 -----------------------
float control2;
float old_control2 = 0;
float error2;
float error_old2 = 0;
float error_old_old2 = 0; 
int control2int = 0;

float KP_2 = 30;
float KI_2 = 9.5;
float KD_2 = 0.01;



const int CMD_MAX = 500;   // max command magnitude you send to Sabertooth (adjust)
const int CMD_MIN = -250;

const float sampleTime_s = 0.010f; // sample time in seconds (match your loop; 10 ms = 0.01)

float clip(float value, float minVal, float maxVal) {
  if (value > maxVal) return maxVal;
  if (value < minVal) return minVal;
  return value;
}

// CONFIGURANDO INTERRUPCIONES
void doEncoder0A()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos--;
  } else {
    encoder0Pos++;
  }
}

void doEncoder0B()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

void doEncoder1A()
{
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos--;
  } else {
    encoder1Pos++;
  }
}

void doEncoder1B()
{
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
}


void setup() {
  // Configurar Encoders
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // Incluir una resistencia de pullup en le entrada
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // Incluir una resistencia de pullup en le entrada
  pinMode(encoder1PinA, INPUT);
  digitalWrite(encoder1PinA, HIGH);       // Incluir una resistencia de pullup en le entrada
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinB, HIGH);       // Incluir una resistencia de pullup en le entrada
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0A, CHANGE);  // encoder 0 PIN A
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoder0B, CHANGE);  // encoder 0 PIN B
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);  // encoder 1 PIN A
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);  // encoder 1 PIN B

  // Configurar Serial port
  Serial.begin(115200);                   // Inicializar el puerto serial (Monitor Serial)
  Serial.println("start");
  Serial1.begin(9600);                  // Comunicacion serial sabertooth

  // Configurar limit switch
  pinMode(LIM0_PIN, INPUT_PULLUP);   // NC to GND → reads LOW normally
  pinMode(LIM1_PIN, INPUT_PULLUP);
  lim0WasPressed = (digitalRead(LIM0_PIN) == LOW);
  lim1WasPressed = (digitalRead(LIM1_PIN) == LOW);

  // Servo 1
  //servo1.attach(9);
  //servo1.write(s1pos1);
  // delay(1000);

  // Servo 2
  // servo2.attach(8);
  // delay(1000);
  // servo2.write(60);
  // delay(2000);
  // servo2.write(75);
  // delay(2000);
  // servo2.write(60);
}

char msgEnd = ';';
bool reading = false;
String instruccion;
bool newMsg = false;

String readBuff() {
  String buffArray;
  //int i = 0;

  while (Serial.available() > 0) { //Entro a este while mientras exista algo en el puerto serial
    char buff = Serial.read(); //Leo el byte entrante
    if (buff == msgEnd) {
      newMsg = true;
      reading = false;
      break; //Si el byte entrante coincide con mi delimitador, me salgo del while
    } else {
      buffArray += buff; //Si no, agrego el byte a mi string para construir el mensaje
      reading = true;
    }
    // delay(6.7);
  }

  return buffArray;  //Retorno el mensaje
}


void loop() {
  if (Serial.available()){
    instruccion = readBuff();
    if (newMsg){
      Serial.print("Instruccion: ");
      Serial.println(instruccion);
    }
  }

  if ((micros() - time_ant) >= Period)
    {
      if (firstPID) {
        // inicializar errores previos para evitar derivative kick
        error_old1 = error1;
        error_old_old1 = error1;
        error_old2 = error2;
        error_old_old2 = error2;
        firstPID = false;
      }

      if (homing){
        // Rutina de limit switches
        bool lim0Pressed = (digitalRead(LIM0_PIN) == LOW);
        bool lim1Pressed = (digitalRead(LIM1_PIN) == LOW);
        lim0WasPressed = lim0Pressed;
        lim1WasPressed = lim1Pressed;
        // Serial.print("lim0Pressed: ");
        // Serial.print(lim0WasPressed);
        // Serial.print(",  ");
        // Serial.print("lim1Pressed: ");
        // Serial.print(lim1WasPressed);
        // Serial.println(",  ");

        if (lim0WasPressed == 0 && homing_m1){
          // Movemos motor 1 para arriba suave
          control1int = homing_m1_speed;
        }
        else{
          // M1 llego a su cero absoluto.
          control1int = 0;
          homing_m1 = false;
        }
        
        if (lim1WasPressed == 0 && homing_m2){
          // Movemos motor 1 para arriba suave
          control2int = homing_m2_speed;
        }
        else{
          // M1 llego a su cero absoluto.
          control2int = 0;
          homing_m2 = false;
        }


      }

      newtime = micros();


      // Actualizando Informacion de los encoders
      noInterrupts();
      long pos0 = encoder0Pos;
      long pos1 = encoder1Pos;
      interrupts();
      newposition0 = pos0;
      newposition1 = pos1;

      // Calculando Velocidad del motor en unidades de RPM
      // float rpm = 31250;
      // vel0 = (float)(newposition0 - oldposition0) * rpm / (newtime - time_ant);     //RPM
      // vel1 = (float)(newposition1 - oldposition1) * rpm / (newtime - time_ant);     //RPM
      oldposition0 = newposition0;
      oldposition1 = newposition1;
      revM0 = float(newposition0) / float(divM0);
      revM1 = float(newposition1) / float(divM1);
      degM0 = revM0 * 360;  // Grados a los que está eslabon 0
      degM1 = revM1 * 360;  // Grados a los que está eslabon 1

      // PID Control
      if (PID_control){
        // --- Motor 0 PID ---
        error1 = setpoint0 - degM0;
        if (fabs(error1) < pos_tol){  // deadband
          error1 = 0.0;
        }

        control1 = old_control1 + (KP_1 + dt * KI_1 + KD_1 / dt) * error1 + (-KP_1 - (2 * KD_1) / dt) * error_old1 + (KD_1 / dt) * error_old_old1;
        control1 = clip(control1, CMD_MIN, CMD_MAX); // Clipping
        control1int = int(control1);

        // --- Motor 1 PID (same pattern) ---
        error2 = setpoint1 - degM1;
        if (fabs(error2) < pos_tol){  // deadband
          error2 = 0.0;
        }

        control2 = old_control2 + (KP_2 + dt * KI_2 + KD_2 / dt) * error2 + (-KP_2 - (2 * KD_2) / dt) * error_old2 + (KD_2 / dt) * error_old_old2;
        control2 = clip(control2, CMD_MIN, CMD_MAX);
        control2int = int(control2);

        // Motor command a sabertooth.
        String cmd1 = "M1:" + String(control1int);
        String cmd2 = "M2:" + String(control2int);
        Serial1.println(cmd2);
        Serial1.println(cmd1);

        // Actualización variables para siguiente ciclo
        error_old_old1 = error_old1;
        error_old1 = error1;
        old_control1 = control1;
        error_old_old2 = error_old2;
        error_old2 = error2;
        old_control2 = control2;
      }

      String cmd1 = "M1:" + String(control1int);
      String cmd2 = "M2:" + String(control2int);
      Serial1.println(cmd2);
      Serial1.println(cmd1);

      if (print_control){
        // Reportar datos
        Serial.print("pos 1: ");
        Serial.print(degM0);
        Serial.print(",  ");
        Serial.print("pos 2: ");
        Serial.print(degM1);
        Serial.print(",  ");
        Serial.print("output 1: ");
        Serial.print(control1int);
        Serial.print(",  ");
        Serial.print("output 2: ");
        Serial.print(control2int);
        Serial.println(",  ");
      }


      // Servo 1
      if (s1pos1 > s1pos1limit) {
        s1pos1--;
        servo1.write(s1pos1);
      }

      time_ant = newtime;
    }
}
