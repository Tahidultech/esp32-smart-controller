/*
  ESP32 DEVKITV1 Smart Controller Firmware
  Features:
  - 4 Relay control (via ESP RainMaker App and push buttons)
  - Fan speed (+/-) via App and dedicated buttons
  - IR Receiver: logs HEX codes to Serial
  - LDR sensor: light percentage in app, signal loss counter
  - All relays OFF on boot
  - Reset signal loss counter via App
  - Icons for switches in app

  Pin Mapping:
  -----------
  - Light 1 Relay:      GPIO 4   (D4)
  - Light 2 Relay:      GPIO 5   (D5)
  - Light 3 Relay:      GPIO 18  (D18)
  - Fan Relay:          GPIO 19  (D19)
  - Fan Speed +:        GPIO 21  (D21)
  - Fan Speed -:        GPIO 22  (D22)
  - Button Light1:      GPIO 23  (D23)
  - Button Light2:      GPIO 25  (D25)
  - Button Light3:      GPIO 26  (D26)
  - Button Fan:         GPIO 27  (D27)
  - Button Fan Speed +: GPIO 32  (D32)
  - Button Fan Speed -: GPIO 33  (D33)
  - IR Receiver:        GPIO 15  (D15)
  - LDR Sensor:         GPIO 34  (D34)
*/

#include <Arduino.h>
#include <IRremote.h>
#include <IRutils.h>
#include <WiFi.h>
// ESP RainMaker Arduino library
#include <ESP32RainMaker.h>

// Pin Definitions
#define RELAY1 4    // Light 1
#define RELAY2 5    // Light 2
#define RELAY3 18   // Light 3
#define RELAY4 19   // Fan

#define FAN_SPEED_UP 21
#define FAN_SPEED_DOWN 22

#define BTN_RELAY1 23
#define BTN_RELAY2 25
#define BTN_RELAY3 26
#define BTN_FAN     27
#define BTN_FAN_UP  32
#define BTN_FAN_DOWN 33

#define IR_RECEIVER 15
#define LDR_SENSOR 34

// IR
IRrecv irrecv(IR_RECEIVER);
decode_results results;

// LDR
int ldrSignalLoss = 0;
int ldrValue = 0;

// RainMaker devices
SwitchDevice *deviceRelay1, *deviceRelay2, *deviceRelay3, *deviceFan;
SwitchDevice *deviceFanSpeedUp, *deviceFanSpeedDown, *deviceReset;
Device *deviceLDR;

// State
bool relayState[4] = {false, false, false, false};
int fanSpeed = 0; // Simulated

void IRAM_ATTR handleButton(int idx);

void setup() {
  Serial.begin(115200);

  // GPIO Setup
  pinMode(RELAY1, OUTPUT); digitalWrite(RELAY1, LOW);
  pinMode(RELAY2, OUTPUT); digitalWrite(RELAY2, LOW);
  pinMode(RELAY3, OUTPUT); digitalWrite(RELAY3, LOW);
  pinMode(RELAY4, OUTPUT); digitalWrite(RELAY4, LOW);

  pinMode(FAN_SPEED_UP, OUTPUT); digitalWrite(FAN_SPEED_UP, LOW);
  pinMode(FAN_SPEED_DOWN, OUTPUT); digitalWrite(FAN_SPEED_DOWN, LOW);

  pinMode(BTN_RELAY1, INPUT_PULLUP);
  pinMode(BTN_RELAY2, INPUT_PULLUP);
  pinMode(BTN_RELAY3, INPUT_PULLUP);
  // ... rest of your code ...
}
