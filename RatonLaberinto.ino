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
#define BASE_SPEED 115

/* PID */
#define KP 0.095
#define KI 0
#define KD 0.0055
#define YST 2000

int y = 0;
float u = 0;
int errorP = 0;
int errorI = 0;
int errorD = 0;
int previousError = 0;

/* CONTROL REMOTO */
#define REMOTEXY__DEBUGLOG
#define REMOTEXY_MODE__ESP32CORE_BLE

#include <BLEDevice.h>

#define REMOTEXY_BLUETOOTH_NAME "RemoteXY"
#define REMOTEXY_ACCESS_PASSWORD "hola"

#include <RemoteXY.h>

#pragma pack(push, 1)  
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =   // 78 bytes V19 
  { 255,5,0,2,0,71,0,19,0,0,0,65,120,101,108,0,31,2,106,200,
  200,84,1,1,4,0,5,207,26,143,143,16,14,60,60,0,2,26,31,1,
  57,72,57,57,133,21,52,52,0,2,31,0,67,44,164,21,24,85,26,40,
  10,86,2,26,7,43,109,24,24,84,46,40,10,118,64,2,26,2 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t joystick_01_x; // from -100 to 100
  int8_t joystick_01_y; // from -100 to 100
  uint8_t button_01; // =1 if button pressed, else =0, from 0 to 1
  int16_t edit_01; // -32768 .. +32767

    // output variables
  int16_t value_01; // -32768 .. +32767

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

  // Secuencia de inicio
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

void loop() {
  RemoteXYEngine.handler(); 
  handleController();
  //updatePID();
  //changeSpeedWithPWD();
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

void handleController() {
  int speed = constrain(RemoteXY.edit_01, 0, 255);
  RemoteXY.value_01 = speed;
  if (RemoteXY.joystick_01_x == 0) {
    setMotors(speed * RemoteXY.button_01, speed * RemoteXY.button_01);
  } else {
    setMotors((RemoteXY.joystick_01_x * 0.01) * speed * RemoteXY.button_01, (RemoteXY.joystick_01_x * 0.01) * -speed * RemoteXY.button_01);
  }
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

void changeSpeedWithPWD() {
  if (u > 0)
    setMotors(BASE_SPEED, BASE_SPEED - u);
  else
    setMotors(BASE_SPEED + u, BASE_SPEED);
}

/*
void motorTest() {
  setMotors(BASE_SPEED, BASE_SPEED);
  delay(3000);
  setMotors(0, 0);
  delay(3000);
  setMotors(-BASE_SPEED, -BASE_SPEED);
  delay(3000);
  setMotors(0, 0);
  delay(3000);
}

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