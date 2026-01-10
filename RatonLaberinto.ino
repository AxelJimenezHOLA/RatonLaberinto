// Board: Node32s

/* MOTORES */
#define MOTOR_RIGHT_POSITIVE 16
#define MOTOR_RIGHT_NEGATIVE 17
#define MOTOR_RIGHT_A 35
#define MOTOR_RIGHT_B 34
#define MOTOR_LEFT_POSITIVE 4
#define MOTOR_LEFT_NEGATIVE 18
#define MOTOR_LEFT_A 23
#define MOTOR_LEFT_B 19

/* SENSORES INFRARROJOS */
#define IR0 36
#define IR1 33
#define IR2 25
#define IR3 26
#define IR4 27
#define IR5 32
#define IR6 39
#define LED_ON 14

/* OTROS PINES */
#define PUSH 12
#define BATTERY 13
#define LED 5

/* CONFIGURACIONES */
#define FREQUENCY_PWM 4000
#define RESOLUTION_PWM 8
int speed = 0;

/* PID */
int YST = 2000;
double KP = 0.095;
double KI = 0;
double KD = 0.0055;

int y = 0;
double u = 0;
double errorP = 0;
double errorI = 0;
double errorD = 0;
double previousError = 0;

/* CONTROL REMOTO */
#define REMOTEXY__DEBUGLOG
#define REMOTEXY_MODE__ESP32CORE_BLE

#include <BLEDevice.h>

#define REMOTEXY_BLUETOOTH_NAME "RatonLaberinto"

#include <RemoteXY.h>

#pragma pack(push, 1)  
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =   // 114 bytes V19 
  { 255,17,0,0,0,107,0,19,0,0,0,76,97,98,101,114,105,110,116,111,
  65,76,0,69,2,106,200,200,84,1,1,5,0,7,8,78,24,24,44,10,
  63,10,110,64,2,26,2,7,7,5,86,24,24,109,22,40,10,110,64,2,
  26,2,1,7,5,138,24,24,44,22,63,10,110,64,2,26,2,7,7,68,
  33,24,24,109,10,40,10,110,64,2,26,2,1,10,26,110,57,57,44,42,
  105,31,49,4,1,31,79,78,0,31,79,70,70,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  float edit_kp;
  float edit_yst;
  float edit_kd;
  float edit_speed;
  uint8_t pushSwitch_01; // =1 if state is ON, else =0, from 0 to 1

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   

#pragma pack(pop)

void setup() {
  RemoteXY_Init();
  Serial.begin(115200);
  Serial.println("Iniciando");

  // PWM de los motores
  ledcAttach(MOTOR_RIGHT_POSITIVE, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcAttach(MOTOR_RIGHT_NEGATIVE, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcAttach(MOTOR_LEFT_POSITIVE, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcAttach(MOTOR_LEFT_NEGATIVE, FREQUENCY_PWM, RESOLUTION_PWM);

  // Inicializar motores en velocidad 0
  ledcWrite(MOTOR_RIGHT_POSITIVE, 0);
  ledcWrite(MOTOR_RIGHT_NEGATIVE, 0);
  ledcWrite(MOTOR_LEFT_POSITIVE, 0);
  ledcWrite(MOTOR_LEFT_NEGATIVE, 0);

  // Inicializar pines digitales
  pinMode(PUSH, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  pinMode(LED_ON, OUTPUT);

  digitalWrite(LED,1);
  //secuenciaInicio();
}

void loop() {
  RemoteXYEngine.handler(); 
  updateRemote();
  updatePID();
  changeSpeedWithPWM();
}

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  setMotor(MOTOR_LEFT_POSITIVE, MOTOR_LEFT_NEGATIVE, leftMotorSpeed);
  setMotor(MOTOR_RIGHT_POSITIVE, MOTOR_RIGHT_NEGATIVE, rightMotorSpeed);
}

void setMotor(int motorPin1, int motorPin2, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    ledcWrite(motorPin1, speed);
    ledcWrite(motorPin2, 0);
  } else if (speed < 0) {
    ledcWrite(motorPin1, 0);
    ledcWrite(motorPin2, -speed);
  } else {
    ledcWrite(motorPin1, 0);
    ledcWrite(motorPin2, 0);
  }
}

void updateRemote() {
  KP = RemoteXY.edit_kp * RemoteXY.pushSwitch_01;
  KD = RemoteXY.edit_kd * RemoteXY.pushSwitch_01;
  speed = RemoteXY.edit_speed * RemoteXY.pushSwitch_01;
  YST = RemoteXY.edit_yst * RemoteXY.pushSwitch_01;
}

void updatePID() {
  y = analogRead(IR2);

  // Errores calculados
  errorP = YST - y;
  errorI += errorP;
  errorD = errorP - previousError;

  u = KP * errorP + KI * errorI + KD * errorD;
  previousError = errorP;
}

void changeSpeedWithPWM() {
  if (u > 0)
    setMotors(speed, speed - u);
  else
    setMotors(speed + u, speed);
}

void secuenciaInicio() {
  digitalWrite(LED, HIGH);
  while (digitalRead(PUSH)) {
    //irDebugPrint();
    updatePID();
    setMotors(u, -u);
  }

  for (int k = 0; k < 3; k++) {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }
}
/*
void irDebugPrint() {
  digitalWrite(LED_ON, 1);
  Serial.print(analogRead(BATTERY) * 0.000805664062); // El valor equivale a 3.3 / 4096
  Serial.print("\t");
  Serial.print(analogRead(IR0));
  Serial.print("\t");
  Serial.print(analogRead(IR1));
  Serial.print("\t");
  Serial.print(analogRead(IR2));
  Serial.print("\t");
  Serial.print(analogRead(IR3));
  Serial.print("\t");
  Serial.print(analogRead(IR4));
  Serial.print("\t");
  Serial.print(analogRead(IR5));
  Serial.print("\t");
  Serial.println(analogRead(IR6));
  digitalWrite(LED_ON, 0);
}
*/