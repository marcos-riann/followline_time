#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//Enter Line Details
bool isBlackLine = 1;             //keep 1 in case of black line. In case of white line change this to 0
unsigned int lineThickness = 25;  //Enter line thickness in mm. Works best for thickness between 10 & 35
unsigned int numSensors = 5;      // Enter number of sensors as 5 or 7
bool brakeEnabled = 0;

int aenbl = 5;
int aFrente = 7;
int aTras = 6;

int benbl = 11;
int bFrente = 9;
int bTras = 10;

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfSpeed = 60;
int currentSpeed = 30;

float Kp = 0.06;
float Kd = 1.5;
float Ki = 0;

int onLine = 1;
int minValues[6], maxValues[6], threshold[6], sensorValue[6];
bool brakeFlag = 0;

// Variáveis para temporizador
unsigned long startTime;          // Armazena o tempo de início
unsigned long timeLimit = 30000;  // 30 segundos (30000 milissegundos) (segundos x 1000)

void setup() {
  for (int i = 1; i <= 5; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;
  }

  pinMode(aenbl, OUTPUT);
  pinMode(aFrente, OUTPUT);
  pinMode(aTras, OUTPUT);

  pinMode(benbl, OUTPUT);
  pinMode(bFrente, OUTPUT);
  pinMode(bTras, OUTPUT);

  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(9600);
  pinMode(13, OUTPUT);

  lineThickness = constrain(lineThickness, 10, 35);

  delay(2000);
  Serial.println("inicio da calibragem");

  // Realize a calibração movendo o robô sobre a linha e o fundo.
  for (int i = 0; i < 300; i++) {
    calibrate();
    delay(5);
  }

  Serial.println("fim da calibragem");
  delay(2000);

  //codigo abaixo usado para teste de valores

  Serial.println("threshold");
  for (int i = 1; i <= 5; i++) {
    Serial.print(threshold[i]);
    Serial.print(" ");
  }

  //Serial.println();
  //Serial.println("sensorValue");
  //for (int i = 1; i <= 5; i++) {
  //  Serial.print(sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000));
  //  Serial.print(" ");
  //}

  Serial.println();
  Serial.println("minValues");
  for (int i = 1; i <= 5; i++) {
    Serial.print(minValues[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("maxValues");
  for (int i = 1; i <= 5; i++) {
    Serial.print(maxValues[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Inicializa o temporizador
  startTime = millis();
}

void loop() {
  // Verifica se o tempo limite foi atingido (30 segundos)
  if (millis() - startTime >= timeLimit) {
    // Para os motores
    motor(0, 0);
    Serial.println("Tempo limite atingido, robô parado.");
    while (true);  // Pausa o loop indefinidamente
  }

  readLine();
  if (currentSpeed < lfSpeed) currentSpeed++;
  if (onLine == 1) {  // PID LINE FOLLOW
    linefollow();
    digitalWrite(13, HIGH);
    brakeFlag = 0;
  } else {
    digitalWrite(13, LOW);
    if (error > 0) {
      if (brakeEnabled == 1 && brakeFlag == 0) {
        motor(0, 0);
        delay(30);
      }
      motor(-75, 80);
      brakeFlag = 1;
    } else {
      if (brakeEnabled == 1 && brakeFlag == 0) {
        motor(0, 0);
        delay(30);
      }
      motor(75, -80);
      brakeFlag = 1;
    }
  }
}

void motor(int speedA, int speedB) {
  if (speedA < 0) {       // se a velocidade do motor A for menor que zero então ele vai andar para tras
    speedA = 0 - speedA;  // a velocidade passa a ser positiva mas ele vai andar para tras
    digitalWrite(aFrente, LOW);
    digitalWrite(aTras, HIGH);
  } else {
    digitalWrite(aFrente, HIGH);
    digitalWrite(aTras, LOW);
  }
  if (speedB < 0) {       // se a velocidade do motor B for menor que zero então ele vai andar para tras
    speedB = 0 - speedB;  // a velocidade passa a ser positiva mas ele vai andar para tras
    digitalWrite(bFrente, LOW);
    digitalWrite(bTras, HIGH);
  } else {
    digitalWrite(bFrente, HIGH);
    digitalWrite(bTras, LOW);
  }
  analogWrite(aenbl, speedA);
  analogWrite(benbl, speedB);
}

void linefollow() {
  if (numSensors == 5) {
    error = (3 * sensorValue[1] + sensorValue[2] - sensorValue[4] - 3 * sensorValue[5]);
  }
  if (lineThickness > 22) {
    error = error * -1;
  }
  if (isBlackLine) {
    error = error * -1;
  }

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  if (lsp > 80) {
    lsp = 80;
  }
  if (lsp < -75) {
    lsp = -75;
  }
  if (rsp > 80) {
    rsp = 80;
  }
  if (rsp < -75) {
    rsp = -75;
  }
  motor(lsp, rsp);
}

void calibrate() {
  for (int i = 1; i <= 5; i++) {
    if (analogRead(i) < minValues[i]) {
      minValues[i] = analogRead(i);
    }
    if (analogRead(i) > maxValues[i]) {
      maxValues[i] = analogRead(i);
    }
  }

  for (int i = 1; i <= 5; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(analogRead(i));
    Serial.print(" ");
  }
  Serial.println();
}

void readLine() {
  onLine = 0;

  for (int i = 1; i <= 5; i++) {
    sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    if (isBlackLine == 1 && sensorValue[i] > 700) onLine = 1;
    if (isBlackLine == 0 && sensorValue[i] < 700) onLine = 1;
  }
}
