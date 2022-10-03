#include "TBSense.h"

BLESense gBLESense;
time_t gLastTime = 0;

void setup()
{
  Serial.begin(9600);
  // while (!Serial);

  // set LED pin to output mode
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  digitalWrite(LEDR, LOW); // turn the LED on
  digitalWrite(LEDG, LOW); // turn the LED on
  digitalWrite(LEDB, LOW); // turn the LED on

  delay(250); // wait a bit

  digitalWrite(LEDR, HIGH); // turn the LED off
  delay(250);               // wait a bit

  digitalWrite(LEDG, HIGH); // turn the LED off
  delay(250);               // wait a bit

  digitalWrite(LEDB, HIGH); // turn the LED off
  delay(250);               // wait a bit

  // begin initialization
  if (!gBLESense.init())
  {
    Serial.println(F("starting TouchBlueSensor failed!"));

    digitalWrite(LEDG, HIGH); // turn the LED off
    digitalWrite(LEDB, HIGH); // turn the LED off
    while (1)
    {
      digitalWrite(LEDR, HIGH); // turn the LED off
      delay(500);               // wait a bit
      digitalWrite(LEDR, LOW);  // turn the LED on
      delay(500);               // wait a bit
    }
  }

  gLastTime = millis();
}

void loop()
{
  time_t now = millis();
  time_t dt = now - gLastTime;
  gBLESense.update(dt);

  static int frameCount = 0;
  frameCount += 1;
  static int countTime = 0;
  countTime += dt;

  if (countTime > 1000)
  {
    Serial.print(F("FPS "));
    Serial.print(frameCount);
    Serial.print(F(" | time "));
    Serial.print(countTime);
    Serial.print(F(" | SRAM "));
    Serial.println(freeRam());

    countTime = 0;
    frameCount = 0;
  }

  gLastTime = now;
}

//SRAM usage in AVR-based Arduino
// int freeRam()
// {
//   extern int __heap_start, *__brkval;
//   int v;
//   return (int)&v - (__brkval == 0
//                         ? (int)&__heap_start
//                         : (int)__brkval);
// }

extern "C" char* sbrk(int incr);
int freeRam() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}
