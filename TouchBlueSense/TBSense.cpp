#include "TBSense.h"

LSM6DS3 BLESense::gSensor(I2C_MODE, LSM6DS3_C_ACC_GYRO_WHO_AM_I);
Adafruit_NeoPixel BLESense::gLedStrip(24/*number of leds*/, PIN_A0/*first pin*/, NEO_GRB + NEO_KHZ800);
BLEService BLESense::ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service
BLEByteCharacteristic BLESense::switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic BLESense::brightnessCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic BLESense::saturationCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

#define ledPin LEDR
#define int1Pin PIN_LSM6DS3TR_C_INT1
uint8_t tapCount = 0; // Amount of received interrupts
uint8_t prevTapCount = 0; // Interrupt Counter from last loop
const byte manufacturerData[] = { 0x54, 0x6f, 0x75, 0x63, 0x68, 0x42, 0 };

int config_touch_detect(LSM6DS3 &sensor_LSM6DS3);

BLESense::BLESense()
{

}

bool BLESense::init()
{
    bool ok = true;
    
    ok = ok && setupBle();
    ok = ok && setupStrip();
    ok = ok && setupSensor();
    
    return ok;
}

void BLESense::update(time_t dt)
{
    updateBle(dt);
    updateStrip(dt);
    updateSensor(dt);
}

bool BLESense::setupBle()
{
  // begin initialization
  if (!BLE.begin()) {
      Serial.println(F("starting Bluetooth® Low Energy module failed!"));
      return false;
  }
 
  // set advertised parameters
  BLE.setDeviceName("TBSense");
  BLE.setLocalName("LED");
  // appearance (0x015 << 6 | 0x12) = 0x0552
  // category: Sensor 0x015 | Sub-category: Multi-Sensor 0x12
  BLE.setAppearance(0x0552);
  BLE.setManufacturerData(manufacturerData, sizeof(manufacturerData) / sizeof(manufacturerData[0]));
  BLE.setAdvertisingInterval(3200); // 2000ms (3200 * 0.625)
  BLE.setAdvertisedService(ledService);
 
  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);
  ledService.addCharacteristic(brightnessCharacteristic);
  ledService.addCharacteristic(saturationCharacteristic);
 
  // add service
  BLE.addService(ledService);
 
  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);
  brightnessCharacteristic.writeValue(0x40);
  saturationCharacteristic.writeValue(0xff);

  // Build scan response data packet
  BLEAdvertisingData scanData;
  // Set parameters for scan response packet
  scanData.setLocalName("TouchBlue Sense");
  // Copy set parameters in the actual scan response packet
  BLE.setScanResponseData(scanData);

  // Build advertising data packet
  BLEAdvertisingData advData;
  // Set parameters for advertising packet
  advData.setManufacturerData(manufacturerData, sizeof(manufacturerData));
  advData.setAdvertisedService(ledService);
  //advData.setAdvertisedServiceData(0xfff0, serviceData, sizeof(serviceData));
  // Copy set parameters in the actual advertising packet
  BLE.setAdvertisingData(advData);
 
  // start advertising
  BLE.advertise();
  
  Serial.println(F("BLE LED Peripheral"));
  return true;
}

bool BLESense::setupStrip()
{
  gLedStrip.begin();
  // turn off
  gLedStrip.show();

  return true;
}

bool BLESense::setupSensor()
{
  gSensor.settings.gyroEnabled = 0; // Gyro currently not used, disabled to save power 
  if (gSensor.begin() != 0) {
      Serial.println(F("Device error"));
      return false;
  } else {
      Serial.println(F("Device OK!"));
  }
  //Configure LSM6DS3 as touch sensor
  if (0 != config_touch_detect(gSensor)) {
      Serial.println(F("Configure touch detect fail!"));
      return false;
  }
  
  return true;
}

void BLESense::updateBle(time_t deltaTime) {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();
 
  // if a central is connected to peripheral:
  if (central.connected()) {
    digitalWrite(LEDB, HIGH);  // turn the LED off
 
    // while the central is still connected to peripheral:
    //while (central.connected()) {
        if (switchCharacteristic.written()) {
          if (switchCharacteristic.value()) {   
            Serial.println(F("LED on"));
            digitalWrite(ledPin, LOW); // changed from HIGH to LOW       
          } else {                       
            Serial.println(F("LED off"));
            digitalWrite(ledPin, HIGH); // changed from LOW to HIGH     
          }
        }
    //}

  } else {
    // show blue when not connected
    digitalWrite(LEDB, LOW);   // turn the LED on
  }
}

void BLESense::updateStrip(time_t deltaTime)
{
    static uint16_t current_hue = 0;
    current_hue = current_hue + 43;
    gLedStrip.rainbow(current_hue, 1, saturationCharacteristic.value(), brightnessCharacteristic.value());
    gLedStrip.show();
}

void BLESense::updateSensor(time_t deltaTime)
{
  static time_t timeSinceLastTap = 0;
  timeSinceLastTap += deltaTime;
  if (timeSinceLastTap > 100)
    digitalWrite(LEDG, HIGH);   // turn green LED off
  
  int delta = tapCount - prevTapCount;
  if (delta == 0)
    return;
  prevTapCount += delta;
    
  Serial.print(F("Tap detected "));
  Serial.println(delta);

  digitalWrite(LEDG, LOW);   // turn green LED on
  timeSinceLastTap = 0;
}

// taken from https://forum.seeedstudio.com/t/xiao-ble-sense-lsm6ds3-int1-single-tap-interrupt/264206/6
void int1ISR()
{
  tapCount++;
}

int config_touch_detect(LSM6DS3 &gSensor) {
  //Error accumulation variable
  uint8_t errorAccumulator = 0;

  uint8_t dataToWrite;  //Temporary variable

  //Setup the accelerometer******************************
  dataToWrite = 0; //Start Fresh!
  //dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz; // 0x01
  //dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;    // 0x00
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;// 0x60

  // Now, write the patched together data
  errorAccumulator += gSensor.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  // Set the ODR bit
  //errorAccumulator += gSensor.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
  //dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

  // Enable tap detection on X, Y, Z axis, but do not latch output

  errorAccumulator += gSensor.writeRegister( LSM6DS3_ACC_GYRO_TAP_CFG1, 0x8E ); // SLOPE_FDS + TAP_*_EN
  
  // Set tap threshold
  // Write 0Ch into TAP_THS_6D
  errorAccumulator += gSensor.writeRegister( LSM6DS3_ACC_GYRO_TAP_THS_6D, /*0x03*/0x8C );

  // Set Duration, Quiet and Shock time windows
  // Write 7Fh into INT_DUR2
  errorAccumulator += gSensor.writeRegister( LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F );
  
  // Single & Double tap enabled (SINGLE_DOUBLE_TAP = 1)
  // Write 80h into WAKE_UP_THS
  errorAccumulator += gSensor.writeRegister( LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80 );
  
  // Single tap interrupt driven to INT1 pin -- enable latch
  // Write 08h into MD1_CFG
  errorAccumulator += gSensor.writeRegister( LSM6DS3_ACC_GYRO_MD1_CFG, 0x48 );

  pinMode(int1Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(int1Pin), int1ISR, RISING);
    
  return errorAccumulator;
}
