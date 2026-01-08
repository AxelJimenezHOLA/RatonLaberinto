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
#define MOTOR_SPEED 127

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
  Serial.println(digitalRead(PUSH));
  setMotors(MOTOR_SPEED, MOTOR_SPEED);
  delay(3000);
  setMotors(0, 0);
  delay(3000);
  setMotors(-MOTOR_SPEED, -MOTOR_SPEED);
  delay(3000);
  setMotors(0, 0);
  delay(3000);
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