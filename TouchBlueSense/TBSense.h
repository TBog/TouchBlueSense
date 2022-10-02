#ifndef _TB_SENSE_H_
#define _TB_SENSE_H_

#include <LSM6DS3.h>           // accelerometer, gyroscope
#include <Adafruit_NeoPixel.h> // WS2812 led strip
#include <ArduinoBLE.h>

class BLESense
{
  // we need an instance of class LSM6DS3 to enable tap/touch detection
  static LSM6DS3 gSensor;
  static Adafruit_NeoPixel gLedStrip;
  static BLEService ledService;
  static BLEByteCharacteristic switchCharacteristic;
  static BLEByteCharacteristic brightnessCharacteristic;
  static BLEByteCharacteristic saturationCharacteristic;

  static BLEService accelService;
  static BLEByteCharacteristic accelRangeCharacteristic;
  static BLEWordCharacteristic accelBandWidthCharacteristic;
  static BLEWordCharacteristic accelSampleRateCharacteristic;

public:
  BLESense();

  bool init();
  void update(time_t deltaTime);

private:
  bool setupBle();
  bool setupStrip();
  bool setupSensor();

  void updateBle(time_t deltaTime);
  void updateStrip(time_t deltaTime);
  void updateSensor(time_t deltaTime);
};

#endif //_TB_SENSE_H_
