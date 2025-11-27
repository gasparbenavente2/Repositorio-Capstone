#include <Servo.h>

// Servo 1
Servo servo1;
int s1pos1 = 100;               // current angle
int s1pos1limit = 100;

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
float homing_theta1 = 78.8; // Angulo al que queda theta 1 despues del homing 78.8
float homing_theta2 = 68.1; // 68.1

// Control macro
bool homing = false; // Cuando se prende el robot, se mueve lentamente hasta llegar a los enconders
float PID_control = !homing;
bool print_control = false; // imprimir señales en terminal
float setpoint0 =  0;     // eslabon 1
float setpoint1 =  0;  // eslabon 2

// homing
bool homing_m1 = true;  // No modificar
bool homing_m2 = true;  // No modificar
int homing_m1_speed = 320;
int homing_m2_speed = -60;


// Variables control PID motoes
int firstPID = false;
float pos_tol = 0.1; // Valor al que se considera alcanzado el angulo
int max_integrador = 100; // No implementado todavia

// ------------------- M1 -----------------------
float control1 = 0;
float old_control1 = 0;
float error1 = 0;
float error_old1 = 0;
float error_old_old1 = 0;
float degM0_old1;
float degM0_old_old1;
int control1int = 0;

float KP_1 = 12;
float KI_1 = 2; // 2.0
float KD_1 = 0.35; // 0.35
//float KD_1 = 0.00;

// ------------------- M2 -----------------------
float control2 = 0;
float old_control2 = 0;
float error2 = 0;
float error_old2 = 0;
float error_old_old2 = 0; 
int control2int = 0;
float degM1_old2;
float degM1_old_old2;

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
  servo1.attach(9);
  servo1.write(100);

  delay(1000);

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
    delay(6.7);
  }

  return buffArray;  //Retorno el mensaje
}

int s_com_para_90 = 132;
int servo_horizon_to_command(int theta1, int theta2, int theta_horizon){
  // Funcion recibe angulo del python: Angulo de pistola con respecto a la horizontal
  // Y retorna el comando que se le debe mandar al servo dependiendo del estado del robot.
  

  return s_com_para_90 - theta_horizon + theta1 + theta2 - 180;


}


void loop() {
  if (Serial.available()){
    instruccion = readBuff();
    
    if (instruccion[0] == 'A' && newMsg) {
      String estado = instruccion.substring(1, 5);

      if (estado == "GOTO"){

        PID_control = true;
        homing = false;

        float q1 = instruccion.substring(6, 12).toFloat() - homing_theta1;
        float q2 = instruccion.substring(13, 19).toFloat() - homing_theta2;
        int q3 = instruccion.substring(20, 24).toInt();

        Serial.print(q1);
        Serial.print(q2);

        s1pos1 = servo_horizon_to_command(homing_theta1, homing_theta2, q3);
        Serial.println(s1pos1);

        error1 = 0;
        error2 = 0;
        setpoint0 = q1;
        setpoint1 = q2;
      }
      else if (estado == "HOME"){
        homing = true;
        PID_control = false;
      }
    }
  }

  if ((micros() - time_ant) >= Period)
    {
      bool lim0Pressed = (digitalRead(LIM0_PIN) == LOW);
      bool lim1Pressed = (digitalRead(LIM1_PIN) == LOW);
      bool lim0WasPressed = lim0Pressed;
      bool lim1WasPressed = lim1Pressed;

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
      degM0 = degM0; //+ homing_theta1;
      degM1 = degM1; // + homing_theta2;


      if (firstPID) {
        // inicializar errores previos para evitar derivative kick
        error1 = setpoint0 - degM0;
        error_old1 = error1;
        error_old_old1 = error1;
        degM0_old1 = degM0;
        degM0_old_old1 = degM0;

        error2 = setpoint1 - degM1;
        error_old2 = error2;
        error_old_old2 = error2;
        firstPID = false;
      }

      

      if (homing){

        if (lim0WasPressed == 0 && homing_m1){
          // Movemos motor 1 para arriba suave
          control1int = homing_m1_speed;
        }
        else{
          // M1 llego a su cero absoluto.
          control1int = 0;
          // degM0 = homing_theta1;
          // degM0_old1 = degM0;
          // degM0_old_old1 = degM0;
          homing_m1 = false;
        }
        
        if (lim1WasPressed == 0 && homing_m2){
          // Movemos motor 1 para arriba suave
          control2int = homing_m2_speed;
        }
        else{
          // M1 llego a su cero absoluto.
          control2int = 0;
          // degM1 = homing_theta2;
          homing_m2 = false;
        }

        if (!homing_m1 && !homing_m2 && homing){
          Serial.println("AHOME");
          homing = false;
        }


      }

      

      // PID Control
      if (PID_control){
        //// --- Motor 0 PID ---
        
        error1 = setpoint0 - degM0;
        if (fabs(error1) < pos_tol){  // deadband
          error1 = 0.0;
        }
        
        float dP_1 = KP_1 * (error1 - error_old1);
        float dI_1 = KI_1 * dt * error1;
        float dD_1 = -KD_1 * (degM0 - 2.0f * degM0_old1 + degM0_old_old1) / dt;
        control1  = old_control1 + dP_1 + dI_1 + dD_1;

        // control1 = old_control1 + (KP_1 + dt * KI_1 + KD_1 / dt) * error1 + (-KP_1 - (2 * KD_1) / dt) * error_old1 + (KD_1 / dt) * error_old_old1;
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
        //cmd1 = "M1:" + String(control1int);
        //cmd2 = "M2:" + String(control2int);



        // Actualización variables para siguiente ciclo
        error_old_old1 = error_old1;
        error_old1 = error1;
        old_control1 = control1;
        error_old_old2 = error_old2;
        error_old2 = error2;
        old_control2 = control2;
        degM0_old_old1 = degM0_old1;
        degM0_old1 = degM0;


      }

      // Si se apretan lim switch frenar robot

      //if (lim0WasPressed || lim1WasPressed){
      //  control1int = 0;
      //  control2int = 0;
      //}
      servo1.write(s1pos1);
      String cmd1 = "M1:" + String(control1int);
      String cmd2 = "M2:" + String(control2int);
      //Serial1.println(cmd2);
      //Serial1.println(cmd1);
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
        Serial.print(",  ");
        Serial.print("command servo: ");
        Serial.print(s1pos1);
        Serial.println(",  ");
      }


      // Servo 1
      // if (s1pos1 > s1pos1limit) {
      //  s1pos1--;
      //   servo1.write(s1pos1);
      // }

      time_ant = newtime;
    }
}
