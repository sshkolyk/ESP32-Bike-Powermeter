#include "globals.h"
#include "HX711.h"
#include "BLEPowerCSC.cpp" //TODO can this be swapped out for an .h?
#include "Adafruit_MPU6050.h"
#include <Wire.h>
#include "driver/adc.h"
// calibration for get Kg force
//#define CALIBRATION_FACTOR 32.46
#define CALIBRATION_FACTOR 34.4
#define LOADCELL_OFFSET 768451
//in meters
#define CRUNK_LENGTH 0.175
//for translate grammforce to Nm
#define TRANSFORM_FORCE_CONSTANT 0.00980665

//todo comment in production
#define DEBUG
// 1. HX711 circuit wiring
#define LOADCELL_SDA_PIN 23u
#define LOADCELL_SCK_PIN 19u
//MPU6050 wiring
#define GYRO_SDA_PIN 5u
#define GYRO_SCK_PIN 4u
#define GYRO_INT_PIN GPIO_NUM_34
#define LED_PIN 22

Adafruit_MPU6050 mpu;

BLEPowerCSC *bluetooth = new BLEPowerCSC();
HX711 loadCell;


void initScale() {
  loadCell.begin(LOADCELL_SDA_PIN, LOADCELL_SCK_PIN); //init HX711
  loadCell.set_scale();
  loadCell.set_offset(LOADCELL_OFFSET);

  loadCell.tare(255u);                                  //Reset to zero

  loadCell.set_scale(CALIBRATION_FACTOR);             //set calibration scale
  delay(300);
  loadCell.power_down();
#ifdef DEBUG
  Serial.println("HX711 initialized");
#endif
}

void initGyro() {
  Wire.begin(GYRO_SDA_PIN, GYRO_SCK_PIN);
  // Try to initialize!
  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
#ifdef DEBUG
  Serial.println("MPU6050 Found!");
#endif

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  //260 or 184, or 94, or 44 or 21 or 10 or 5
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.enableSleep(true);
  pinMode(GYRO_INT_PIN, INPUT_PULLUP); //pullup interrupt ping
}

void setup()
{
  for (int i = 0; i < 20; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
  }
  adc_power_off(); //disable ADC
  
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial.printf("CPU freq = %dMHz\n", getCpuFrequencyMhz());
  initScale();
  initGyro();
  
  bluetooth->initialize();
  Serial.println("bluetooth initialized");
}





uint64_t lastCSCEventTimeStamp = millis(), currentCSCEventTimeStamp = 0;
sensors_event_t a, g, temp;

void setRotations() {
  /* Get new sensor events with the readings */
  mpu.getEvent(&a, &g, &temp);
  //in rotations per second z axis
  bluetooth->rotationSpeed = fabs(g.gyro.z / 6.283);
  currentCSCEventTimeStamp = millis();
  bluetooth->cumulativeRevolutions += abs(currentCSCEventTimeStamp - lastCSCEventTimeStamp) * bluetooth->rotationSpeed / 1000;
#ifdef DEBUG
  //Serial.printf("delta = %lf,\t speed= %lf\n", abs(currentCSCEventTimeStamp - lastCSCEventTimeStamp) * bluetooth->rotationSpeed / 1000, bluetooth->rotationSpeed);
  //Serial.printf("heading = %f\n", g.gyro.heading);
#endif
  lastCSCEventTimeStamp = currentCSCEventTimeStamp;
  
}

void setTorqueMoment() {
  if (loadCell.is_ready()) {  
    float units = loadCell.get_units();
    bluetooth->torqueMoment = units * TRANSFORM_FORCE_CONSTANT * CRUNK_LENGTH * 2 ; //2 - because of 2 cranks
#ifdef DEBUG
    Serial.printf("Nm = %f,\toffset=%lu,\tForce = %f\n", bluetooth->torqueMoment, loadCell.get_offset(), units);
    delay(100);
#endif
  }
}

bool deviceConnected = false;
bool oldDeviceConnected = false;

void loop()
{
  // notify changed value
  if (deviceConnected)
  {
    setRotations();
    setTorqueMoment();
    bluetooth->sendData(); //the release-ready power send function, for an already calibrated and working device.
    delay(49); // the minimum is 3ms according to official docs
    digitalWrite(LED_PIN, LOW);
    delay(1); //indicate data update
    digitalWrite(LED_PIN, HIGH);
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    mpu.enableSleep(true);
    loadCell.power_down();
    bluetooth->startBroadcast();
    oldDeviceConnected = deviceConnected;
  } 
  // connecting
  else if (deviceConnected && !oldDeviceConnected)
  {
    mpu.enableSleep(false);
    loadCell.power_up();
    lastCSCEventTimeStamp = millis();
    bluetooth->cumulativeRevolutions = 0;
    bluetooth->lastSendCSCTimeStamp = bluetooth->getCSCSendTimeStamp();
    bluetooth->lastSendCSCValue = 0;
    bluetooth->rotationSpeed = 0;
    bluetooth->torqueMoment = 0;
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
  else if (!deviceConnected) {
    delay(15000);
    if (deviceConnected) return;
    /*mpu.enableSleep(false);
    esp_sleep_enable_ext0_wakeup(GYRO_INT_PIN, HIGH);
    Serial.println("going to deep sleep");
    esp_deep_sleep_start();*/
    esp_sleep_enable_timer_wakeup(5000000u); //microseconds
    esp_light_sleep_start();
  }
}