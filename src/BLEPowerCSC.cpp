#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>
#include "globals.h"

#define CYCLING_POWER_SERVICE_UUID "00001818-0000-1000-8000-00805F9B34FB"
#define POWER_CHARACTERISTIC_UUID "00002A63-0000-1000-8000-00805F9B34FB"
#define SENSORPOS_CHARACTERISTIC_UUID "00002A5D-0000-1000-8000-00805F9B34FB"
#define POWERFEATURE_CHARACTERISTIC_UUID "00002A65-0000-1000-8000-00805F9B34FB"

#define LED_PIN 22



class MyServerCallbacks : public BLEServerCallbacks
{
private:

    void onConnect(BLEServer *pServer) //todo add a reference to deviceconnected
    {
        deviceConnected=true;
        digitalWrite(LED_PIN, LOW);
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected=false;
        digitalWrite(LED_PIN, HIGH);
    }
};

class BLEPowerCSC
{ //a class for holding bluetooth-related code
private:
    BLEServer *pServer = NULL;
    //Power Characteristics
    BLECharacteristic *pCharacteristicPower = NULL; //the power reading itself
    BLECharacteristic *pCharacteristicSensorPos = NULL;
    BLECharacteristic *pCharacteristicPowerFeature = NULL;
    //CSC ONES
    

    uint8_t powerTxValue[8]; //the BLE-Compliant, flagged, ready to transmit power and cadence value 

    //connection indicators
    bool deviceConnected = false;
    bool oldDeviceConnected = false;

public:
    const uint8_t powerFlags[2] = {0x20, 0};
    uint64_t lastSendCSCTimeStamp = getCSCSendTimeStamp();
    uint32_t lastSendCSCValue = 0;
    double_t cumulativeRevolutions = 0;
    double_t rotationSpeed = 0;
    double_t torqueMoment = 0;
    uint16_t power = 0;
    BLEPowerCSC()
    { // a constructor
    }

    void initialize()
    {
        digitalWrite(LED_PIN, HIGH);
        
        // Create the BLE Device
        BLEDevice::init("ESP32PowerMeter"); // weirdly enough names with spaces do not seem to work

        // Create the BLE Server
        pServer = BLEDevice::createServer();
        pServer->setCallbacks(new MyServerCallbacks());

        // Create the BLE Service
        BLEService *pService = pServer->createService(CYCLING_POWER_SERVICE_UUID);

        // Create the needed BLE Characteristics
        pCharacteristicPower = pService->createCharacteristic(
            POWER_CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_NOTIFY);

        pCharacteristicSensorPos = pService->createCharacteristic(
            SENSORPOS_CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_READ);

        pCharacteristicPowerFeature = pService->createCharacteristic(
            POWERFEATURE_CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_READ);

        //CSC CHARACTERISTICS
        
        // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
        // Create a BLE Descriptor
        pCharacteristicPower->addDescriptor(new BLE2902());

        // Start the service
        pService->start();

        uint8_t posvalue[] = {6}; // right crank
        pCharacteristicSensorPos->setValue(posvalue, 1);

        //ALL FEATURE SETTING
        uint8_t powerFeature[] = {0x08, 0, 0, 0};
        pCharacteristicPowerFeature->setValue(powerFeature, 4);
        powerTxValue[0] = powerFlags[0]; //flags field in power message
        powerTxValue[1] = powerFlags[1]; //

        lastSendCSCValue = 0; //reset revolutions counter

        // Start advertising
        BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(CYCLING_POWER_SERVICE_UUID); //todo does it report cadence even if it is not advertised
        pAdvertising->setScanResponse(false);
        pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
        BLEDevice::startAdvertising();
    }

    void sendData()
    {
        power = int(fabsf(torqueMoment) * rotationSpeed);
        uint32_t revolutions = int(cumulativeRevolutions); //change timestamp only if revolutions counter increased
        if (revolutions > lastSendCSCValue) {
            lastSendCSCValue = revolutions;
            lastSendCSCTimeStamp = getCSCSendTimeStamp();
        }

        powerTxValue[2] = power & 0xff;
        powerTxValue[3] = (power >> 8) & 0xff;
        powerTxValue[4] = revolutions & 0xff;
        powerTxValue[5] = (revolutions >> 8) & 0xff;
        powerTxValue[6] = lastSendCSCTimeStamp & 0xff;
        powerTxValue[7] = (lastSendCSCTimeStamp >> 8) & 0xff;
        

        pCharacteristicPower->setValue(powerTxValue, 8);
        pCharacteristicPower->notify();
    }

    uint64_t getCSCSendTimeStamp() {
        uint64_t extra_time = rotationSpeed > 0 ? //correction for extra partial revolution
            1000 * (cumulativeRevolutions - lastSendCSCValue) / rotationSpeed :
            0;
        
        return (millis() - extra_time) * 1024 / 1000;
    }

    void startBroadcast()
    {
        delay(100);                  // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
    }
};

