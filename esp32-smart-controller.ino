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
  pinMode(BTN_FAN, INPUT_PULLUP);
  pinMode(BTN_FAN_UP, INPUT_PULLUP);
  pinMode(BTN_FAN_DOWN, INPUT_PULLUP);

  // IR
  irrecv.enableIRIn();

  // ESP RainMaker
  Node my_node;
  my_node = RMaker.initNode("ESP32 Smart Controller");

  // Relays
  deviceRelay1 = new SwitchDevice("Light 1", &relayState[0]);
  deviceRelay2 = new SwitchDevice("Light 2", &relayState[1]);
  deviceRelay3 = new SwitchDevice("Light 3", &relayState[2]);
  deviceFan    = new SwitchDevice("Fan", &relayState[3]);
  // Fan Speed
  deviceFanSpeedUp   = new SwitchDevice("Fan Speed +", nullptr);
  deviceFanSpeedDown = new SwitchDevice("Fan Speed -", nullptr);

  // LDR
  deviceLDR = new Device("LDR", "esp.device.ldr", DEVICE_TYPE_SENSOR);
  Param *ldrParam = new Param("Light %", "esp.param.light", value(0), PROP_FLAG_READ);
  deviceLDR->addParam(ldrParam);

  // Reset
  deviceReset = new SwitchDevice("Reset Counter", nullptr);

  // Add devices
  RMaker.addDevice(*deviceRelay1);
  RMaker.addDevice(*deviceRelay2);
  RMaker.addDevice(*deviceRelay3);
  RMaker.addDevice(*deviceFan);
  RMaker.addDevice(*deviceFanSpeedUp);
  RMaker.addDevice(*deviceFanSpeedDown);/*
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
  pinMode(BTN_FAN, INPUT_PULLUP);
  pinMode(BTN_FAN_UP, INPUT_PULLUP);
  pinMode(BTN_FAN_DOWN, INPUT_PULLUP);

  // IR
  irrecv.enableIRIn();

  // ESP RainMaker
  Node my_node;
  my_node = RMaker.initNode("ESP32 Smart Controller");

  // Relays
  deviceRelay1 = new SwitchDevice("Light 1", &relayState[0]);
  deviceRelay2 = new SwitchDevice("Light 2", &relayState[1]);
  deviceRelay3 = new SwitchDevice("Light 3", &relayState[2]);
  deviceFan    = new SwitchDevice("Fan", &relayState[3]);
  // Fan Speed
  deviceFanSpeedUp   = new SwitchDevice("Fan Speed +", nullptr);
  deviceFanSpeedDown = new SwitchDevice("Fan Speed -", nullptr);

  // LDR
  deviceLDR = new Device("LDR", "esp.device.ldr", DEVICE_TYPE_SENSOR);
  Param *ldrParam = new Param("Light %", "esp.param.light", value(0), PROP_FLAG_READ);
  deviceLDR->addParam(ldrParam);

  // Reset
  deviceReset = new SwitchDevice("Reset Counter", nullptr);

  // Add devices
  RMaker.addDevice(*deviceRelay1);
  RMaker.addDevice(*deviceRelay2);
  RMaker.addDevice(*deviceRelay3);
  RMaker.addDevice(*deviceFan);
  RMaker.addDevice(*deviceFanSpeedUp);
  RMaker.addDevice(*deviceFanSpeedDown);
  RMaker.addDevice(*deviceLDR);
  RMaker.addDevice(*deviceReset);

  RMaker.setWriteCallback(write_callback);

  // WiFi Init (credentials provisioned via RainMaker)
  WiFi.mode(WIFI_STA);

  RMaker.start();
}

void loop() {
  // IR Receiving
  if (irrecv.decode(&results)) {
    Serial.print("IR HEX: 0x");
    Serial.println(results.value, HEX);
    irrecv.resume();
  }

  // LDR Read
  int ldrRaw = analogRead(LDR_SENSOR);
  int ldrPercent = map(ldrRaw, 0, 4095, 0, 100);
  static unsigned long lastLdrUpdate = 0;
  if (millis() - lastLdrUpdate > 1000) {
    deviceLDR->updateParam("Light %", value(ldrPercent));
    lastLdrUpdate = millis();
  }
  // Example signal loss detection
  if (ldrRaw < 50) ldrSignalLoss++;

  // Button handling (debounce omitted for brevity, ideally use interrupts)
  if (!digitalRead(BTN_RELAY1)) { toggleRelay(0); delay(200);}
  if (!digitalRead(BTN_RELAY2)) { toggleRelay(1); delay(200);}
  if (!digitalRead(BTN_RELAY3)) { toggleRelay(2); delay(200);}
  if (!digitalRead(BTN_FAN))    { toggleRelay(3); delay(200);}
  if (!digitalRead(BTN_FAN_UP))   { fanSpeedUp(); delay(200);}
  if (!digitalRead(BTN_FAN_DOWN)) { fanSpeedDown(); delay(200);}
}

void toggleRelay(int idx) {
  relayState[idx] = !relayState[idx];
  digitalWrite(RELAY1 + idx, relayState[idx] ? HIGH : LOW);
  switch (idx) {
    case 0: deviceRelay1->update(relayState[0]); break;
    case 1: deviceRelay2->update(relayState[1]); break;
    case 2: deviceRelay3->update(relayState[2]); break;
    case 3: deviceFan->update(relayState[3]); break;
  }
}

void fanSpeedUp() {
  fanSpeed++;
  deviceFanSpeedUp->update(true);
  digitalWrite(FAN_SPEED_UP, HIGH);
  delay(100);
  digitalWrite(FAN_SPEED_UP, LOW);
  deviceFanSpeedUp->update(false);
}

void fanSpeedDown() {
  fanSpeed--;
  deviceFanSpeedDown->update(true);
  digitalWrite(FAN_SPEED_DOWN, HIGH);
  delay(100);
  digitalWrite(FAN_SPEED_DOWN, LOW);
  deviceFanSpeedDown->update(false);
}

// ESP RainMaker Write Callback
void write_callback(Device *device, Param *param, const esp_param_val_t val, const esp_rmaker_param_type_t type) {
  if (device == deviceRelay1) {
    relayState[0] = val.val.b;
    digitalWrite(RELAY1, relayState[0] ? HIGH : LOW);
    deviceRelay1->update(relayState[0]);
  } else if (device == deviceRelay2) {
    relayState[1] = val.val.b;
    digitalWrite(RELAY2, relayState[1] ? HIGH : LOW);
    deviceRelay2->update(relayState[1]);
  } else if (device == deviceRelay3) {
    relayState[2] = val.val.b;
    digitalWrite(RELAY3, relayState[2] ? HIGH : LOW);
    deviceRelay3->update(relayState[2]);
  } else if (device == deviceFan) {
    relayState[3] = val.val.b;
    digitalWrite(RELAY4, relayState[3] ? HIGH : LOW);
    deviceFan->update(relayState[3]);
  } else if (device == deviceFanSpeedUp) {
    if (val.val.b) fanSpeedUp();
  } else if (device == deviceFanSpeedDown) {
    if (val.val.b) fanSpeedDown();
  } else if (device == deviceReset) {
    if (val.val.b) ldrSignalLoss = 0;
    deviceReset->update(false);
  }
}
  RMaker.addDevice(*deviceLDR);
  RMaker.addDevice(*deviceReset);

  RMaker.setWriteCallback(write_callback);

  // WiFi Init (credentials provisioned via RainMaker)
  WiFi.mode(WIFI_STA);

  RMaker.start();
}

void loop() {
  // IR Receiving
  if (irrecv.decode(&results)) {
    Serial.print("IR HEX: 0x");
    Serial.println(results.value, HEX);
    irrecv.resume();
  }

  // LDR Read
  int ldrRaw = analogRead(LDR_SENSOR);
  int ldrPercent = map(ldrRaw, 0, 4095, 0, 100);
  static unsigned long lastLdrUpdate = 0;
  if (millis() - lastLdrUpdate > 1000) {
    deviceLDR->updateParam("Light %", value(ldrPercent));
    lastLdrUpdate = millis();
  }
  // Example signal loss detection
  if (ldrRaw < 50) ldrSignalLoss++;

  // Button handling (debounce omitted for brevity, ideally use interrupts)
  if (!digitalRead(BTN_RELAY1)) { toggleRelay(0); delay(200);}
  if (!digitalRead(BTN_RELAY2)) { toggleRelay(1); delay(200);}
  if (!digitalRead(BTN_RELAY3)) { toggleRelay(2); delay(200);}
  if (!digitalRead(BTN_FAN))    { toggleRelay(3); delay(200);}
  if (!digitalRead(BTN_FAN_UP))   { fanSpeedUp(); delay(200);}
  if (!digitalRead(BTN_FAN_DOWN)) { fanSpeedDown(); delay(200);}
}

void toggleRelay(int idx) {
  relayState[idx] = !relayState[idx];
  digitalWrite(RELAY1 + idx, relayState[idx] ? HIGH : LOW);
  switch (idx) {
    case 0: deviceRelay1->update(relayState[0]); break;
    case 1: deviceRelay2->update(relayState[1]); break;
    case 2: deviceRelay3->update(relayState[2]); break;
    case 3: deviceFan->update(relayState[3]); break;
  }
}

void fanSpeedUp() {
  fanSpeed++;
  deviceFanSpeedUp->update(true);
  digitalWrite(FAN_SPEED_UP, HIGH);
  delay(100);
  digitalWrite(FAN_SPEED_UP, LOW);
  deviceFanSpeedUp->update(false);
}

void fanSpeedDown() {
  fanSpeed--;
  deviceFanSpeedDown->update(true);
  digitalWrite(FAN_SPEED_DOWN, HIGH);
  delay(100);
  digitalWrite(FAN_SPEED_DOWN, LOW);
  deviceFanSpeedDown->update(false);
}

// ESP RainMaker Write Callback
void write_callback(Device *device, Param *param, const esp_param_val_t val, const esp_rmaker_param_type_t type) {
  if (device == deviceRelay1) {
    relayState[0] = val.val.b;
    digitalWrite(RELAY1, relayState[0] ? HIGH : LOW);
    deviceRelay1->update(relayState[0]);
  } else if (device == deviceRelay2) {
    relayState[1] = val.val.b;
    digitalWrite(RELAY2, relayState[1] ? HIGH : LOW);
    deviceRelay2->update(relayState[1]);
  } else if (device == deviceRelay3) {
    relayState[2] = val.val.b;
    digitalWrite(RELAY3, relayState[2] ? HIGH : LOW);
    deviceRelay3->update(relayState[2]);
  } else if (device == deviceFan) {
    relayState[3] = val.val.b;
    digitalWrite(RELAY4, relayState[3] ? HIGH : LOW);
    deviceFan->update(relayState[3]);
  } else if (device == deviceFanSpeedUp) {
    if (val.val.b) fanSpeedUp();
  } else if (device == deviceFanSpeedDown) {
    if (val.val.b) fanSpeedDown();
  } else if (device == deviceReset) {
    if (val.val.b) ldrSignalLoss = 0;
    deviceReset->update(false);
  }
}
