#include "TBSense.h"
#include "BlinkAnim.h"
#include "LoadingAnim.h"

LSM6DS3 BLESense::gSensor(I2C_MODE, LSM6DS3_C_ACC_GYRO_WHO_AM_I);
Adafruit_NeoPixel BLESense::gLedStrip(24 /*number of leds*/, PIN_A0 /*first pin*/, NEO_GRB + NEO_KHZ800);
BLEService BLESense::ledService("54B10000-5442-6F67-9000-CC505EFFCD37"); // Bluetooth® Low Energy LED Service
BLEByteCharacteristic BLESense::gameStateCharacteristic("54B10008-5442-6F67-9000-CC505EFFCD37", BLERead | BLEWrite);
BLEByteCharacteristic BLESense::switchCharacteristic("54B10001-5442-6F67-9000-CC505EFFCD37", BLERead | BLEWrite);
BLEByteCharacteristic BLESense::brightnessCharacteristic("54B10002-5442-6F67-9000-CC505EFFCD37", BLERead | BLEWrite);
BLEByteCharacteristic BLESense::saturationCharacteristic("54B10003-5442-6F67-9000-CC505EFFCD37", BLERead | BLEWrite);
BLEService BLESense::accelService("54B20000-5442-6F67-9000-CC505EFFCD37");
BLEByteCharacteristic BLESense::accelRangeCharacteristic("54B10004-5442-6F67-9000-CC505EFFCD37", BLERead | BLEWrite);
BLEWordCharacteristic BLESense::accelBandWidthCharacteristic("54B10005-5442-6F67-9000-CC505EFFCD37", BLERead | BLEWrite);
BLEWordCharacteristic BLESense::accelSampleRateCharacteristic("54B10006-5442-6F67-9000-CC505EFFCD37", BLERead | BLEWrite);
BLEIntCharacteristic BLESense::tapCountCharacteristic("54B10007-5442-6F67-9000-CC505EFFCD37", BLERead | BLENotify);

#define ledPin LEDR
#define int1Pin PIN_LSM6DS3TR_C_INT1

// Game State Characteristic
#define GSC_RAINBOW 255
#define GSC_LOADING 254
#define GSC_TOUCH_NOTHING 0
#define GSC_TOUCH_READY 1
#define GSC_TOUCH_ERROR 2
#define GSC_TOUCH_VALID 3
#define GSC_TOUCH_COUNTDOWN 4

int tapCount = 0;     // Amount of received interrupts
int prevTapCount = 0; // Interrupt Counter from last loop
const byte manufacturerData[] = {0x54, 0x6f, 0x75, 0x63, 0x68, 0x42, 0};

int config_touch_detect(LSM6DS3 &sensor_LSM6DS3);

BLESense::BLESense()
    : mGameState(GSC_LOADING), m_pGameStateAnim(nullptr)
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
  if (!BLE.begin())
  {
    Serial.println(F("starting Bluetooth® Low Energy module failed!"));
    return false;
  }

  // set the initial value for the characeristic:
  gameStateCharacteristic.writeValue(mGameState);
  switchCharacteristic.writeValue(0);
  brightnessCharacteristic.writeValue(0x40);
  saturationCharacteristic.writeValue(0xff);
  accelRangeCharacteristic.writeValue(2);        // 2, 4, 8, 16
  accelBandWidthCharacteristic.writeValue(50);   // 50, 100, 200, 400
  accelSampleRateCharacteristic.writeValue(833); // 13,26,52,104,208,416,833,1660,3330,6660,13330
  tapCountCharacteristic.writeValue(prevTapCount);

  // add the characteristic(s) to the service(s)
  ledService.addCharacteristic(gameStateCharacteristic);
  ledService.addCharacteristic(switchCharacteristic);
  ledService.addCharacteristic(brightnessCharacteristic);
  ledService.addCharacteristic(saturationCharacteristic);
  accelService.addCharacteristic(accelRangeCharacteristic);
  accelService.addCharacteristic(accelBandWidthCharacteristic);
  accelService.addCharacteristic(accelSampleRateCharacteristic);
  accelService.addCharacteristic(tapCountCharacteristic);

  // add service(s)
  BLE.addService(ledService);
  BLE.addService(accelService);

  // set advertised parameters
  BLE.setDeviceName("TBSense");
  BLE.setLocalName("LED");
  // appearance (0x015 << 6 | 0x12) = 0x0552
  // category: Sensor 0x015 | Sub-category: Multi-Sensor 0x12
  BLE.setAppearance(0x0552);
  BLE.setAdvertisingInterval(3200); // 2000ms (3200 * 0.625)

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
  // advData.setAdvertisedServiceData(0xfff0, serviceData, sizeof(serviceData));
  //  Copy set parameters in the actual advertising packet
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

  gSensor.settings.accelEnabled = 1;
  gSensor.settings.accelBandWidth = accelBandWidthCharacteristic.value();   // 50, 100, 200, 400
  gSensor.settings.accelRange = accelRangeCharacteristic.value();           // 2,4,8,16
  gSensor.settings.accelSampleRate = accelSampleRateCharacteristic.value(); // 13,26,52,104,208,416,833,1660,3330,6660,13330
  gSensor.settings.accelODROff = 1;                                         // 0=auto BandWidth filter  1=BW filter is determined by `accelBandWidth`

  if (switchCharacteristic.value())
  {
    gSensor.settings.accelBandWidth = 50;   // 50, 100, 200, 400
    gSensor.settings.accelSampleRate = 833; // 13,26,52,104,208,416,833,1660,3330,6660,13330
  }

  gSensor.settings.tempEnabled = 0;

  if (gSensor.begin() != 0)
  {
    Serial.println(F("Device error"));
    return false;
  }
  else
  {
    Serial.println(F("Device OK!"));
  }
  // Configure LSM6DS3 as touch sensor
  if (0 != config_touch_detect(gSensor))
  {
    Serial.println(F("Configure touch detect fail!"));
    return false;
  }

  // reset tap count
  tapCount = prevTapCount = 0;
  return true;
}

void BLESense::updateBle(time_t deltaTime)
{
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central.connected())
  {
    digitalWrite(LEDB, HIGH); // turn the LED off

    // while the central is still connected to peripheral:
    // while (central.connected()) {
    if (switchCharacteristic.written())
    {
      if (switchCharacteristic.value())
      {
        Serial.println(F("LED on"));
        digitalWrite(ledPin, LOW); // changed from HIGH to LOW
      }
      else
      {
        Serial.println(F("LED off"));
        digitalWrite(ledPin, HIGH); // changed from LOW to HIGH
      }
      if (setupSensor())
      {
        Serial.println(F("tap sensor reset"));
      }
      else
      {
        Serial.println(F("ERROR: tap sensor reset failed!"));
      }
    }
    if (accelRangeCharacteristic.written() || accelBandWidthCharacteristic.written() || accelSampleRateCharacteristic.written())
    {
      if (setupSensor())
      {
        Serial.println(F("tap sensor reset"));
      }
      else
      {
        Serial.println(F("ERROR: tap sensor reset failed!"));
      }
    }
    //}
  }
  else
  {
    // show blue when not connected
    digitalWrite(LEDB, LOW); // turn the LED on
  }
}

void BLESense::updateStrip(time_t deltaTime)
{
  const auto gameState = gameStateCharacteristic.value();
  const bool gameStateChanged = mGameState != gameState;
  mGameState = gameState;

  // make sure we remove the animation if the game state changed
  if (m_pGameStateAnim && gameStateChanged)
  {
    delete m_pGameStateAnim;
    m_pGameStateAnim = nullptr;
  }

  switch (gameState)
  {
  case GSC_RAINBOW:
  {
    static uint16_t current_hue = 0;
    current_hue = current_hue + 43;
    gLedStrip.rainbow(current_hue, 1, saturationCharacteristic.value(), brightnessCharacteristic.value());
    gLedStrip.show();
    break;
  }

  case GSC_LOADING:
    if (!m_pGameStateAnim)
      m_pGameStateAnim = new LoadingAnim(0x3F3F00);
    break;

  case GSC_TOUCH_ERROR:
    if (!m_pGameStateAnim)
      m_pGameStateAnim = new BlinkAnim(0x3F0000);
    break;

  case GSC_TOUCH_VALID:
    if (!m_pGameStateAnim)
      m_pGameStateAnim = new BlinkAnim(0x003F00);
    break;

  case GSC_TOUCH_READY:
    gLedStrip.fill(0x00003F);
    gLedStrip.show();
    break;

  case GSC_TOUCH_NOTHING:
  default:
    gLedStrip.clear();
    gLedStrip.show();
    break;
  }

  if (m_pGameStateAnim)
  {
    m_pGameStateAnim->update(deltaTime, gLedStrip);
    gLedStrip.show();
  }
}

void BLESense::updateSensor(time_t deltaTime)
{
  static time_t timeSinceLastTap = 0;
  timeSinceLastTap += deltaTime;
  if (timeSinceLastTap > 100)
    digitalWrite(LEDG, HIGH); // turn green LED off

  int delta = tapCount - prevTapCount;
  if (delta == 0)
    return;
  prevTapCount += delta;

  Serial.print(F("Tap(s) detected "));
  Serial.print(delta);
  Serial.print(F(" | time="));
  Serial.print(timeSinceLastTap);
  Serial.print(F(" | total count="));
  Serial.println(tapCount);

  digitalWrite(LEDG, LOW); // turn green LED on
  timeSinceLastTap = 0;
  tapCountCharacteristic.writeValue(tapCount);
}

// taken from https://forum.seeedstudio.com/t/xiao-ble-sense-lsm6ds3-int1-single-tap-interrupt/264206/6
void int1ISR()
{
  tapCount++;
}

int config_touch_detect(LSM6DS3 &gSensor)
{
  // Error accumulation variable
  uint8_t errorAccumulator = 0;

  // Enable tap detection on X, Y, Z axis, but do not latch output
  errorAccumulator += gSensor.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x8E); // SLOPE_FDS + TAP_*_EN

  // Set tap threshold
  // Write 0Ch into TAP_THS_6D
  errorAccumulator += gSensor.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, /*0x03*/ 0x8C);

  // Set Duration, Quiet and Shock time windows
  // Write 7Fh into INT_DUR2
  errorAccumulator += gSensor.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F);

  // Single & Double tap enabled (SINGLE_DOUBLE_TAP = 1)
  // Write 80h into WAKE_UP_THS
  errorAccumulator += gSensor.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);

  // Single tap interrupt driven to INT1 pin -- enable latch
  // Write 08h into MD1_CFG
  errorAccumulator += gSensor.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x48);

  pinMode(int1Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(int1Pin), int1ISR, RISING);

  return errorAccumulator;
}
