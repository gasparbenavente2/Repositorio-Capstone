// Definición de PINs.
// Encoders
#define encoder0PinA  21    // Yellow
#define encoder0PinB  2     // White
#define encoder1PinA  20    // Yellow
#define encoder1PinB  3     // White        // Green GND, Blue 5V

// Limit switches
#define LIM0_PIN 24         // Green
#define LIM1_PIN 25         // Green
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

// Variables control PID motoes
// ------------------- M1 -----------------------
float control1;
float old_control1 = 0;
float error1;
float error_old1 = 0;
float error_old_old1 = 0;

float KP_1 = 0;
float KI_1 = 0;
float KD_1 = 0;

// ------------------- M2 -----------------------
float control2;
float old_control2 = 0;
float error2;
float error_old2 = 0;
float error_old_old2 = 0;

float KP_2 = 1;
float KI_2 = 0;
float KD_2 = 0;


long setpoint0 = 0;   
long setpoint1 = -0;

// ---------------------------------------------------------------
// Transformar setpoint a grados. 
// long counts_per_rev = divM0; // you already have divM0
// float desired_deg = 90.0;
// setpoint0 = (long)( (desired_deg/360.0) * counts_per_rev );
// ---------------------------------------------------------------
// HACER: Hacer para 0 y 1. Hacer para position en loop principal

const int CMD_MAX = 100;   // max command magnitude you send to Sabertooth (adjust)
const int CMD_MIN = -50;

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
  Serial1.begin(119200);                  // Comunicacion serial sabertooth

  // Configurar limit switch
  pinMode(LIM0_PIN, INPUT_PULLUP);   // NC to GND → reads LOW normally
  pinMode(LIM0_PIN, INPUT_PULLUP);
  lim0WasPressed = (digitalRead(LIM0_PIN) == LOW);
  lim1WasPressed = (digitalRead(LIM1_PIN) == LOW);
}

void loop() {
  if ((micros() - time_ant) >= Period)
    {
      newtime = micros();

      // Check limit switches
      bool lim0Pressed = (digitalRead(LIM0_PIN) == LOW);
      bool lim1Pressed = (digitalRead(LIM1_PIN) == LOW);

      if (lim0Pressed && !lim0WasPressed) {
        // reset encoder0 safely
        noInterrupts();
        encoder0Pos = 0;
        interrupts();
        oldposition0 = 0;
        // integ0 = 0.0;        // reset integrator to avoid jump
        // prevErr0 = 0.0;
        Serial.println("LIM0 pressed");
      }

      if (lim1Pressed && !lim1WasPressed) {
        // reset encoder1 safely
        noInterrupts();
        encoder1Pos = 0;
        interrupts();
        oldposition1 = 0;
        // integ1 = 0.0;        // reset integrator to avoid jump
        // prevErr1 = 0.0;
        Serial.println("LIM1 pressed");
      }

      lim0WasPressed = lim0Pressed;
      lim1WasPressed = lim1Pressed;

      // Actualizando Informacion de los encoders
      noInterrupts();
      long pos0 = encoder0Pos;
      long pos1 = encoder1Pos;
      interrupts();
      newposition0 = pos0;
      newposition1 = pos1;

      // Calculando Velocidad del motor en unidades de RPM
      float rpm = 31250;
      vel0 = (float)(newposition0 - oldposition0) * rpm / (newtime - time_ant);     //RPM
      vel1 = (float)(newposition1 - oldposition1) * rpm / (newtime - time_ant);     //RPM
      oldposition0 = newposition0;
      oldposition1 = newposition1;
      revM0 = float(newposition0) / float(divM0);
      revM1 = float(newposition1) / float(divM1);
      degM0 = revM0 * 360;  // Grados a los que está eslabon 0
      degM1 = revM1 * 360;  // Grados a los que está eslabon 1

      // --- Motor 0 PID ---
      error1 = setpoint0 - degM0;
      control1 = old_control1 + (KP_1 + dt * KI_1 + KD_1 / dt) * error1 + (-KP_1 - (2 * KD_1) / dt) * error_old1 + (KD_1 / dt) * error_old_old1;
      control1 = clip(control1, CMD_MIN, CMD_MAX); // Clipping

      // --- Motor 1 PID (same pattern) ---
      error2 = setpoint1 - degM1;
      control2 = old_control2 + (KP_2 + dt * KI_2 + KD_2 / dt) * error2 + (-KP_2 - (2 * KD_2) / dt) * error_old2 + (KD_2 / dt) * error_old_old2;
      control2 = clip(control2, CMD_MIN, CMD_MAX);

      // Mandar mensajes a sabertooth
      control2 = -0;
      Serial1.print("M1:"); Serial1.println(control1);
      Serial1.print("M2:"); Serial1.println(control2);


      // Actualización variables para siguiente ciclo
      error_old_old1 = error_old1;
      error_old1 = error1;
      old_control1 = control1;
      error_old_old2 = error_old2;
      error_old2 = error2;
      old_control2 = control2;

      // Reportar datos
      Serial.print("pos0: ");
      Serial.print(newposition0);
      Serial.print(",  ");
      Serial.print("pos1: ");
      Serial.print(newposition1);
      Serial.print(",  ");
      Serial.print("degM0: ");
      Serial.print(degM0);
      Serial.print(",  ");
      Serial.print("degM1: ");
      Serial.print(degM1);
      Serial.print(",");
      Serial.print("output0: ");
      Serial.print(control1);
      Serial.print(",  ");
      Serial.print("output1: ");
      Serial.print(control2);
      Serial.println(",  ");

      time_ant = newtime;
    }
}
