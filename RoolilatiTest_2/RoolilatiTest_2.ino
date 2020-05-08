#include <EEPROM.h>

#define pwmPin 8
#define dirPin 12
#define buttonL 11
#define buttonR 10
#define tempoPotePin A1
#define pausePotePin A2
#define profileButton 2
#define CLK 31
#define DO 33
#define CSn 35

int pause = 0;
bool dirValue = 0;
int pwmValue = 0;
bool buttonStateL = 0;
bool buttonStateR = 0;
bool lastButtonStateL = 0;
bool lastButtonStateR = 0;
long counterButtonL = 0;
long counterButtonR = 0;
int eeAddress = 0;
volatile int profileState = 0;

int encoderMin;
int encoderMax;
int nextValue;
int nextPWMValue;


void setup() {
  eeAddress = 0;
  EEPROM.get(eeAddress, counterButtonL);
  eeAddress += sizeof(long);
  EEPROM.get(eeAddress, counterButtonR);
  TCCR1B = TCCR1B & 0b11111000 | 1;


  pinMode(buttonR, INPUT_PULLUP);
  pinMode(buttonL, INPUT_PULLUP);
  pinMode(tempoPotePin, INPUT);
  pinMode(pausePotePin, INPUT);
  pinMode(profileButton, INPUT_PULLUP);
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(CSn, OUTPUT);// Chip select
  pinMode(CLK, OUTPUT);// Serial clock
  pinMode(DO, INPUT_PULLUP);// Serial data IN/OUT
  digitalWrite(CSn, HIGH);
  digitalWrite(CLK, HIGH);

  attachInterrupt(digitalPinToInterrupt(2), profileChange, FALLING);

  Serial.begin(9600);
  Serial.print("Left button count: ");
  Serial.println(counterButtonL);
  Serial.print("Right button count: ");
  Serial.println(counterButtonR);


  rangeSetup();
  Serial.println("encoder");
  Serial.println(encoderMin);
  Serial.println(encoderMax);
  nextValue = (encoderMin + encoderMax) / 2;
}

void loop() {
  pwmValue = analogRead(tempoPotePin) / 4;
  pause = analogRead(pausePotePin) * 5;

  Serial.println(profileState);
  //  Serial.println(pwmValue);



  switch (profileState) {
    case 0:
      profile0();
      break;
    case 1:
      profile1();
      break;
    default:
      analogWrite(pwmPin, 0);
      Serial.println(readEncoder());
  }


  eeAddress = 0;
  EEPROM.put(eeAddress, counterButtonL);
  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, counterButtonR);
}


////////


void profileChange() {
  if (digitalRead(profileButton) == LOW) {
    delayMicroseconds(5000);
    if (digitalRead(profileButton) == LOW) {
      if (profileState == 2) profileState = 0;
      else profileState++;
    }
  }
}


void profile0() { // Simple back-and-forth. Speed from potentiometer.
  if (readEncoder() < encoderMin - 200 || readEncoder() > encoderMax + 200) rangeSetup(); // see segab?

  Serial.println(readEncoder());
  if (checkButtonL() && (readEncoder() < encoderMin + 100)) {
    dirValue = 1;
  }
  if (checkButtonR() && (readEncoder() > encoderMax - 200)) {
    dirValue = 0;
    analogWrite(pwmPin, 0);
    delay(pause);
  }

  digitalWrite(dirPin, dirValue);
  analogWrite(pwmPin, pwmValue);
}

void profile1() { // Random

  if (checkButtonL() | checkButtonR()) rangeSetup();
  Serial.print("next    ");
  Serial.println(nextValue);
  Serial.print("current ");
  Serial.println(readEncoder());
  nextPWMValue = pwmValue;
  if (abs(nextValue - readEncoder()) < 30) {
    getNextValue();

    analogWrite(pwmPin, 0);
    delay(pause);
  }
  if (nextValue > readEncoder()) dirValue = 1;
  else dirValue = 0;

  digitalWrite(dirPin, dirValue);
  analogWrite(pwmPin, nextPWMValue);
}


bool checkButtonL() {
  buttonStateL = !digitalRead(buttonL);
  if (buttonStateL != lastButtonStateL) {
    if (buttonStateL) {
      delayMicroseconds(5000);
      if (buttonStateL) {
        counterButtonL++;
        lastButtonStateL = buttonStateL;
        return true;
      }
    }
  }
  lastButtonStateL = buttonStateL;
  return false;
}

bool checkButtonR() {
  buttonStateR = !digitalRead(buttonR);
  if (buttonStateR != lastButtonStateR) {
    if (buttonStateR) {
      delayMicroseconds(5000);
      if (buttonStateR) {
        counterButtonR++;
        lastButtonStateR = buttonStateR;
        return true;
      }
    }
  }
  lastButtonStateR = buttonStateR;
  return false;
}


void rangeSetup() { // Get min and max value for encoder

  while (!checkButtonL()) {
    dirValue = 0;
    encoderMin = readEncoder();
    digitalWrite(dirPin, dirValue);
    analogWrite(pwmPin, 100);
  }

  while (!checkButtonR()) {
    dirValue = 1;
    encoderMax = readEncoder();
    digitalWrite(dirPin, dirValue);
    analogWrite(pwmPin, 100);
  }
  dirValue = 0;
  digitalWrite(dirPin, dirValue);
  analogWrite(pwmPin, 100);
  delay(200);
  analogWrite(pwmPin, 0);

}

void getNextValue() {
  nextValue = random(encoderMin+50, encoderMax-50);
  //nextPWMValue = random(70,256);
}

int readEncoder(void) {//read clk cs do from encoder
  int i, dReading;
  char Resolution = 12;
  unsigned int bitStart = 0x0800;
  dReading = 0;
  digitalWrite(CSn, LOW);
  delayMicroseconds(5);
  digitalWrite(CLK, LOW);
  for (i = (Resolution - 1); i >= 0; i--)
  { digitalWrite(CLK, HIGH);
    delayMicroseconds(5);
    if (digitalRead(DO)) dReading |= bitStart;
    digitalWrite(CLK, LOW);
    delayMicroseconds(5);
    bitStart = bitStart >> 1;
    if (i == 0)
    { digitalWrite(CLK, HIGH);
      if (digitalRead(DO)) dReading |= bitStart;
    }
  }
  digitalWrite(CSn, HIGH);
  return dReading;
}
