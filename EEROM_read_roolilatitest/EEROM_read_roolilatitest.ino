#include <EEPROM.h>
int eeAddress = 0;
long counterButtonL;
long counterButtonR;

void setup() {
  eeAddress = 0;
  EEPROM.get(eeAddress, counterButtonL);
  eeAddress += sizeof(long);
  EEPROM.get(eeAddress, counterButtonR);
  Serial.begin(9600);
  Serial.print("Left button count: ");
  Serial.println(counterButtonL);
  Serial.print("Right button count: ");
  Serial.println(counterButtonR);
}

void loop() {
  // put your main code here, to run repeatedly:

}
