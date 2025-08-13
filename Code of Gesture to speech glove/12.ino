#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <MPU6050_tockn.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// MAX30102
MAX30105 particleSensor;
const byte sampleBufferSize = 75;
uint32_t irBuffer[sampleBufferSize];
uint32_t redBuffer[sampleBufferSize];
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

// MPU6050
MPU6050 mpu(Wire);
unsigned long lastGestureTime = 0;

// Fall detection
float prevMagnitude = 1.0;
const float DELTA_MAG_THRESHOLD = 1.2;
const float Z_DROP_THRESHOLD = -0.8;
const int FALL_DEBOUNCE_TIME = 1000;
unsigned long lastFallTime = 0;
bool fallDetected = false;

// Flex Sensor
const int flexPinThumb  = 32;
const int flexPinIndex  = 33;
const int flexPinMiddle = 35;
const float fixedResistor = 10.0;
float thresholdThumb  = 10.0;
float thresholdIndex  = 10.0;
float thresholdMiddle = 5.0;

// BLE
BLECharacteristic *gloveCharacteristic;
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_GLOVE_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"

String getGestureMeaning(int code) {
  switch (code) {
    case 0: return "Stop";
    case 1: return "Hungry";
    case 2: return "Fall Detected";
    case 3: return "Washroom";
    case 4: return "Me";
    case 5: return "You";
    case 6: return "Good";
    case 7: return "Bad";
    case 8: return "Nothing";
    default: return "None";
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  analogSetAttenuation(ADC_11db);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();

  // MPU6050
  mpu.begin();
  mpu.calcGyroOffsets(true);

  // MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    display.setCursor(0, 10);
    display.println("MAX30102 FAIL");
    display.display();
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x2F);
  particleSensor.setPulseAmplitudeIR(0x2F);

  // BLE
  BLEDevice::init("Sensible Wearable");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  gloveCharacteristic = pService->createCharacteristic(
                          CHAR_GLOVE_UUID,
                          BLECharacteristic::PROPERTY_READ |
                          BLECharacteristic::PROPERTY_NOTIFY
                        );
  gloveCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
  Serial.println("📡 BLE Advertising started");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("All sensors OK");
  display.display();
  delay(1000);
}

void loop() {
  // Read MAX30102
  bool validSample = true;
  for (byte i = 0; i < sampleBufferSize; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    if (irBuffer[i] < 5000) {
      validSample = false;
      break;
    }
    particleSensor.nextSample();
  }
  if (validSample) {
    maxim_heart_rate_and_oxygen_saturation(
      irBuffer, sampleBufferSize,
      redBuffer, &spo2, &validSPO2,
      &heartRate, &validHeartRate
    );
  }

  // MPU6050 + Tilt + Fall
  mpu.update();
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();
  float magnitude = sqrt(ax * ax + ay * ay + az * az);
  float deltaMag = abs(magnitude - prevMagnitude);
  prevMagnitude = magnitude;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float roll  = atan2(ay, az) * 180.0 / PI;
  float fz = az;

  // Enhanced tilt orientation
  String motionGesture = detectTiltOrientation(ax, ay, az);
  if (motionGesture != "Level" && millis() - lastGestureTime > 500) {
    Serial.print("Motion: ");
    Serial.println(motionGesture);
    lastGestureTime = millis();
  }

  // Fall Detection
  // Fall Detection (Enhanced Sensitivity)
unsigned long now = millis();
fallDetected = false;
if ((deltaMag > 0.9 && fz < -0.5) || az < -1.2) {  // More sensitive + strong negative Z spike
  if (now - lastFallTime > FALL_DEBOUNCE_TIME) {
    Serial.println("Fall Detected (High Sensitivity)!");
    lastFallTime = now;
    fallDetected = true;
  }
}


  // Flex Sensor
 // Flex Sensor
float middleRes = readFlexResistance(flexPinMiddle);
float indexRes  = readFlexResistance(flexPinIndex);
float thumbRes  = readFlexResistance(flexPinThumb);

Serial.print("Flex Res (kΩ) => ");
Serial.print("Middle: "); Serial.print(middleRes, 2);
Serial.print("  Index: ");  Serial.print(indexRes, 2);
Serial.print("  Thumb: ");  Serial.println(thumbRes, 2);

  bool isMiddleBent = middleRes < thresholdMiddle;
  bool isIndexBent  = indexRes  < thresholdIndex;
  bool isThumbBent  = thumbRes  < thresholdThumb;

  String fingerGesture = "Unknown";
  if (!isMiddleBent && !isIndexBent && !isThumbBent) fingerGesture = "Open Palm";
  else if (isMiddleBent && isIndexBent && isThumbBent) fingerGesture = "Fist";
  else if (!isIndexBent && isMiddleBent && isThumbBent) fingerGesture = "Pointing";
  else if (!isThumbBent && isIndexBent && isMiddleBent) fingerGesture = "Thumbs Up";
  else if (!isIndexBent && !isMiddleBent && isThumbBent) fingerGesture = "Peace Sign";
  else if (isIndexBent && !isMiddleBent && isThumbBent) fingerGesture = "Middle";

  // Gesture Code Mapping
  int gestureCode = -1;
  if (fingerGesture == "Open Palm" && motionGesture == "Tilt Backward") gestureCode = 0;//stop
  else if (fingerGesture == "Fist" && motionGesture == "Palm Up") gestureCode = 1;//hungry
  else if (fallDetected) gestureCode = 2;
  else if (fingerGesture == "Fist" && motionGesture == "Tilt Right") gestureCode = 3;//washroom
  else if (fingerGesture == "Pointing" && motionGesture == "Tilt Backward") gestureCode = 4;//me
  else if (fingerGesture == "Pointing" && motionGesture == "Tilt Right") gestureCode = 5;//you
  else if (fingerGesture == "Thumbs Up" && motionGesture == "Tilt Right") gestureCode = 6;//good
  else if (fingerGesture == "Thumbs Up" && motionGesture == "Tilt Left") gestureCode = 7;//bad
  if (gestureCode == -1) gestureCode = 8;

  String finalMeaning = getGestureMeaning(gestureCode);

  // OLED
  display.clearDisplay();
  if (validSample && validHeartRate && validSPO2 &&
      heartRate > 40 && heartRate < 220 &&
      spo2 > 60 && spo2 <= 100) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("BPM: ");
    display.print(heartRate);
    display.print("  SpO2: ");
    display.print(spo2);
    display.println("%");
    display.setCursor(0, 20);
    display.print("Gesture: ");
    display.println(finalMeaning);
  } else {
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println("please keep your");
    display.setCursor(0, 32);
    display.println("finger on sensor");
  }
  display.display();

  // BLE Send
  if (validSample && validHeartRate && validSPO2 &&
      heartRate > 40 && heartRate < 220 &&
      spo2 > 60 && spo2 <= 100) {
    String bleData = String(gestureCode) + "," + String(heartRate) + "," + String(spo2);
    gloveCharacteristic->setValue(bleData.c_str());
    gloveCharacteristic->notify();
    Serial.println("BLE Sent: " + bleData);
  }

  Serial.print("Finger: "); Serial.println(fingerGesture);
  Serial.print("Motion: "); Serial.println(motionGesture);
  Serial.print("Meaning: "); Serial.println(finalMeaning);
  Serial.println("--------------------------------------\n");

  delay(500);
}

// ------- Helpers --------

String detectTiltOrientation(float ax, float ay, float az) {
  const float tiltThreshold = 0.4;
  ax = -ax;
  az = -az;
  if (az > 0.8) return "Palm Up";
  else if (az < -0.8) return "Palm Down";
  else if (ay > tiltThreshold) return "Tilt Forward";
  else if (ay < -tiltThreshold) return "Tilt Backward";
  else if (ax > tiltThreshold) return "Tilt Right";
  else if (ax < -tiltThreshold) return "Tilt Left";
  return "Level";
}

float readFlexResistance(int pin) {
  int rawADC = analogRead(pin);
  float voltage = rawADC * (3.3 / 4095.0);
  voltage = fmax(voltage, 0.01);
  return fixedResistor * (voltage / (3.3 - voltage)); // kΩ
}
