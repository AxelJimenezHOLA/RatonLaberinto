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
#define BASE_SPEED 80

/* PID */
#define KP 0.09
#define KI 0
#define KD 0.005
#define YST 2000

int yPosition = 0;
int errorP = 0;
int errorI = 0;
int errorD = 0;
double u = 0;


void setup() {
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

  // Secuencia de inicio
  digitalWrite(LED, HIGH);
  while (digitalRead(PUSH));

  for (int k = 0; k < 5; k++) {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }
}

void loop() {
  updatePID();

  // Velocidad de motores con PID
  if (u > 0)
    setMotors(BASE_SPEED, BASE_SPEED - u);
  else
    setMotors(BASE_SPEED + u, BASE_SPEED);
}

/* FUNCIONES PID */
void updatePID() {
  yPosition = analogRead(IR2);

  // Errores calculados
  errorP = YST - yPosition;
  errorI += errorP;
  errorD = errorP - previousError;

  u = KP * errorP + KI * errorI + KD * errorD;
  previousError = errorP;
}

/* FUNCIONES DE MOTOR */
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

/* FUNCIONES DE PRUEBA */
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

void irTest() {
  Serial.print(analogRead(BATTERY) * (3.3 / 4095.0));
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
}