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
//for translate grammforce to Newtons
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

#define SIGNAL_PATH_RESET 0x68
#define I2C_SLV0_ADDR 0x37
#define ACCEL_CONFIG 0x1C
#define MOT_THR 0x1F // Motion detection threshold bits [7:0]
#define MOT_DUR 0x20 // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MOT_DETECT_CTRL 0x69
#define INT_ENABLE 0x38
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68
#define INT_STATUS 0x3A
#define MPU6050_ADDRESS 0x68


void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.begin();
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress); // Put slave register address in Tx buffer
  Wire.write(data); // Put data in Tx buffer
  Wire.endTransmission(); // Send the Tx buffer
}

void initGyroInterrupt() {
  writeByte(MPU6050_ADDRESS, 0x6B, 0x00);
  writeByte(MPU6050_ADDRESS, SIGNAL_PATH_RESET, 0x07); //Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
  writeByte(MPU6050_ADDRESS, I2C_SLV0_ADDR, 0x20); //write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x01); //Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
  writeByte(MPU6050_ADDRESS, MOT_THR, 10); //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).  
  writeByte(MPU6050_ADDRESS, MOT_DUR, 40); //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate  
  writeByte(MPU6050_ADDRESS, MOT_DETECT_CTRL, 0x15); //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )   
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x40); //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.     
  //writeByte(MPU6050_ADDRESS, 0x37, 160); // now INT pin is active low
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
  initGyroInterrupt();
  pinMode(GYRO_INT_PIN, INPUT_PULLUP); //pullup interrupt pin
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setup()
{
  adc_power_off(); //disable ADC
  
  setCpuFrequencyMhz(80); //just for power saving, be careful with this!
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  print_wakeup_reason();
  Serial.printf("CPU freq = %dMHz\n", getCpuFrequencyMhz());
  initScale();
  initGyro();
  
  pinMode(LED_PIN, OUTPUT);
  bluetooth->initialize();
  Serial.println("bluetooth initialized");
  for (int i = 0; i < 20; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
  }
}





uint64_t lastCSCEventTimeStamp = millis(), currentCSCEventTimeStamp = 0;
sensors_event_t a, g, temp;

void setRotations() {
  /* Get new sensor events with the readings */
  mpu.getEvent(&a, &g, &temp);
  //in rotations per second z axis
  bluetooth->rotationRadiansPerSecond = fabs(g.gyro.z);
  currentCSCEventTimeStamp = millis();
  bluetooth->cumulativeRevolutions += abs(currentCSCEventTimeStamp - lastCSCEventTimeStamp) * bluetooth->getRotationsPerSecond() / 1000;
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
    /*Serial.printf("Nm = %f,\toffset=%lu,\tForce = %f\n", bluetooth->torqueMoment, loadCell.get_offset(), units);
    delay(100);*/
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
    bluetooth->sendData(); 
    delay(29); // the minimum is 3ms according to official docs
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
    Serial.println("bluetooth disconnected");
  } 
  // connecting
  else if (deviceConnected && !oldDeviceConnected)
  {
    mpu.enableSleep(false);
    loadCell.power_up();
    lastCSCEventTimeStamp = millis();
    bluetooth->cumulativeRevolutions = 0;
    bluetooth->lastSendCSCValue = 0;
    bluetooth->rotationRadiansPerSecond = 0;
    bluetooth->torqueMoment = 0;
    bluetooth->lastSendCSCTimeStamp = bluetooth->getCSCSendTimeStamp();
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("bluetooth connected");
  }
  else if (!deviceConnected) {
    for (int i = 0; i < 240; i++) {
      delay(500);
      if (deviceConnected) return;
    }
    mpu.enableSleep(false);
    loadCell.power_down();
    esp_sleep_enable_ext0_wakeup(GYRO_INT_PIN, HIGH);
    Serial.println("going to deep sleep");
    digitalWrite(LED_PIN, LOW);
    delay(200);
    esp_deep_sleep_start();
    /*esp_sleep_enable_timer_wakeup(450000u); //microseconds
      esp_light_sleep_start();*/
  }
}