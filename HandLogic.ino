#include <Wire.h>
#include <Adafruit_INA219.h>
#include <SoftwareServo.h>

Adafruit_INA219 ina219;
SoftwareServo crServo;

// RGB LED Pins
const int red1Pin = 9, green1Pin = 7, blue1Pin = 6; // PWM for Blue
const int red2Pin = 12, green2Pin = 10, blue2Pin = 11; // PWM for Blue

// Servo and Sensor Pins
const int servoPin = 8;
const int hallOpenPin = A0;
const int hallClosePin = A1;
const int emgPin = 13;

// Hall Effect Sensor Thresholds
const int hallOpenThreshold = 453;
const int hallOpenReleaseThreshold = hallOpenThreshold + 2;
const int hallCloseThreshold = 442;
const int hallCloseReleaseThreshold = hallCloseThreshold + 2;

bool hallOpenLatched = false;
bool hallOpenReady = true;
bool hallCloseLatched = false;
bool hallCloseReady = true;

// Current sensing
float filteredCurrent = 0.0;
const float alpha = 0.2;
const float currentThreshold = 150.0;

// EMG Debounce Timers
bool emgState = LOW;
bool prevEmgState = LOW;
unsigned long emgStateStartTime = 0;
bool emgTriggerReady = false;

// Control Flags
bool handClosed = false;
bool homingComplete = false;

void setup() {
  Serial.begin(115200);
  Serial.println("System Starting...");

  if (!ina219.begin()) {
    Serial.println("INA219 not detected!");
    while (1);
  }

  crServo.attach(servoPin);

  pinMode(red1Pin, OUTPUT); pinMode(green1Pin, OUTPUT); pinMode(blue1Pin, OUTPUT);
  pinMode(red2Pin, OUTPUT); pinMode(green2Pin, OUTPUT); pinMode(blue2Pin, OUTPUT);

  pinMode(hallOpenPin, INPUT);
  pinMode(hallClosePin, INPUT);
  pinMode(emgPin, INPUT);

  crServo.write(90); // Stop motor at startup
  Serial.println("Ready. Starting homing sequence...");
}

void loop() {
  unsigned long now = millis();

  // Current Reading & Filtering
  float current_mA = ina219.getCurrent_mA() * 100;
  filteredCurrent = alpha * current_mA + (1.0 - alpha) * filteredCurrent;

  // Hall Sensor Values
  int hallOpenValue = analogRead(hallOpenPin);
  int hallCloseValue = analogRead(hallClosePin);

  // LED Feedback

  // LED 1 (PWM Blue) shows current
  int blueBrightness = map(filteredCurrent, 20, currentThreshold, 0, 255);
  blueBrightness = constrain(blueBrightness, 0, 255);
  analogWrite(blue1Pin, blueBrightness);
  analogWrite(red1Pin, 0);
  analogWrite(green1Pin, 0);

  // LED 2 (Blue) lights up if either Hall sensor is active
  bool hallActive = hallOpenLatched || hallCloseLatched;

  analogWrite(blue2Pin, hallActive ? 255 : 0);   // Turn on blue if Hall triggered
  analogWrite(green2Pin, 0);
  analogWrite(red2Pin, 0);

  // Hall Latching Logic
  if (!handClosed) { // Opening
    if (hallOpenValue <= hallOpenThreshold && hallOpenReady) {
      hallOpenLatched = true;
      hallOpenReady = false;
      Serial.println("Open limit triggered.");
    }
    if (hallOpenValue > hallOpenReleaseThreshold) hallOpenReady = true;
  }

  if (handClosed) { // Closing
    if (hallCloseValue <= hallCloseThreshold && hallCloseReady) {
      hallCloseLatched = true;
      hallCloseReady = false;
      Serial.println("Close limit triggered.");
    }
    if (hallCloseValue > hallCloseReleaseThreshold) hallCloseReady = true;
  }

  // HOMING Sequence: Fully open until hall or current spike
  if (!homingComplete) {
    crServo.write(120);  // Fully open direction
    Serial.println("Homing: Opening...");
    handClosed = false;

    if (filteredCurrent >= currentThreshold || hallOpenLatched) {
      crServo.write(90);  // Stop motor
      delay(300);
      homingComplete = true;
      hallOpenLatched = false;
      Serial.println("Homing complete. Waiting for EMG...");
    }

    SoftwareServo::refresh();
    delay(20);
    return; // Don't process EMG until homing is complete
  }

  // EMG Debounce Logic (HOLD HIGH/LOW for 1 second)
  emgState = digitalRead(emgPin);

  if (emgState != prevEmgState) {
    emgStateStartTime = now;
    emgTriggerReady = false;
  } else {
    if ((now - emgStateStartTime) >= 250) {
      if (emgState == HIGH && !emgTriggerReady) {
        Serial.println("EMG Trigger Detected");
        emgTriggerReady = true;

        if (handClosed) {
          // Start opening
          crServo.write(120);
          Serial.println("Opening...");
          handClosed = false;
        } else {
          // Start closing
          crServo.write(60);
          Serial.println("Closing...");
          handClosed = true;
        }
      }

      if (emgState == LOW && (now - emgStateStartTime >= 2000)) {
        emgTriggerReady = false; // allow next ON trigger after long LOW
      }
    }
  }

  prevEmgState = emgState;

  // Stop Logic (Current spike or Hall triggered)
  if (handClosed && (filteredCurrent >= currentThreshold || hallCloseLatched)) {
    crServo.write(90);
    hallCloseLatched = false;
    Serial.println("Stopped (closing)");
  }

  if (!handClosed && (filteredCurrent >= currentThreshold || hallOpenLatched)) {
    crServo.write(90);
    hallOpenLatched = false;
    Serial.println("Stopped (opening)");
  }

  SoftwareServo::refresh();
  delay(20);
}
