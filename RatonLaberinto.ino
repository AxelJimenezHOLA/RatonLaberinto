// Board: Node32s

#define MOTOR_RIGHT_POSITIVE 16
#define MOTOR_RIGHT_NEGATIVE 17
#define MOTOR_RIGHT_A 35
#define MOTOR_RIGHT_B 34
#define MOTOR_LEFT_POSITIVE 4
#define MOTOR_LEFT_NEGATIVE 18
#define MOTOR_LEFT_A 23
#define MOTOR_LEFT_B 19

#define IR0 36
#define IR1 33
#define IR2 25
#define IR3 26
#define IR4 27
#define IR5 32
#define IR6 39
#define LED_ON 14

#define PUSH 12
#define BATTERY 13
#define LED 5

#define FREQUENCY_PWM 4000
#define RESOLUTION_PWM 8
#define BASE_SPEED 220

//PID
#define KP 0.6
#define KI 0
#define KD 2.2
#define YST 700

//Promedio de resultados anteriores
#define WINDOW_SIZE 5
int window[WINDOW_SIZE];
int windowIndex = 0;
long windowSum = 0;

#define RETRASO 220        
#define TIEMPO_DE_CORRECCION 200      
#define RANGO 10     

unsigned long ultimoMovimiento = 0;
unsigned long iniciarArreglo = 0;
bool arreglando = false;
int YP = 0;
/* ================================ */

int y = 0;
float u = 0;

int errorP = 0;
int errorI = 0;
int errorD = 0;
int previousError = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando");

  ledcAttach(MOTOR_RIGHT_POSITIVE, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcAttach(MOTOR_RIGHT_NEGATIVE, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcAttach(MOTOR_LEFT_POSITIVE, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcAttach(MOTOR_LEFT_NEGATIVE, FREQUENCY_PWM, RESOLUTION_PWM);

  setMotors(0, 0);

  pinMode(PUSH, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  //Promedio de sensores
  for (int i = 0; i < WINDOW_SIZE; i++) {
    int initVal = (analogRead(IR1) + analogRead(IR2)) / 2;
    window[i] = initVal;
    windowSum += initVal;
  }

  ultimoMovimiento = millis();
  YP = windowSum / WINDOW_SIZE;

  digitalWrite(LED, HIGH);
  while (digitalRead(PUSH)) {
    irTest();
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
  updatePID();

  //Detección de bloqueo de movimiento
  if (abs(y - YP) > RANGO) {
    ultimoMovimiento = millis();
    YP = y;
  }

  if (!arreglando && millis() - ultimoMovimiento > RETRASO) {
    arreglando = true;
    iniciarArreglo = millis();
  }

  //Corrección de movimiento
  if (arreglando) {
    setMotors(120, 180);

    if (millis() - iniciarArreglo > TIEMPO_DE_CORRECCION) {
      arreglando = false;
      ultimoMovimiento = millis();
    }
    return;
  }
  setMotors(BASE_SPEED + u, BASE_SPEED - u);
}

void updatePID() {
  int raw = (analogRead(IR1) + analogRead(IR2)) / 2;
  //Filtro de ventana 
  windowSum -= window[windowIndex];
  window[windowIndex] = raw;
  windowSum += raw;

  windowIndex++;
  if (windowIndex >= WINDOW_SIZE) windowIndex = 0;

  y = windowSum / WINDOW_SIZE;

  errorP = YST - y;
  errorI += errorP;
  errorD = errorP - previousError;

  u = KP * errorP + KI * errorI + KD * errorD;
  previousError = errorP;
}

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  setMotor(MOTOR_LEFT_POSITIVE, MOTOR_LEFT_NEGATIVE, leftMotorSpeed);
  setMotor(MOTOR_RIGHT_POSITIVE, MOTOR_RIGHT_NEGATIVE, rightMotorSpeed);
}

void setMotor(int motorPin1, int motorPin2, int speed) {
  speed = constrain(speed, -220, 220);

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

void irTest() {
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
  Serial.print(analogRead(IR6));
  Serial.print("\t");
  Serial.println(y);
}
