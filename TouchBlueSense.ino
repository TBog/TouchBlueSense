/*
// Party lights

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LEDR, HIGH);  // turn the LED off
  digitalWrite(LEDG, HIGH);  // turn the LED off
  digitalWrite(LEDB, HIGH);  // turn the LED off

  delay(1000);               // wait for a second
  
  digitalWrite(LEDR, LOW);   // turn the LED on
  delay(1000);               // wait for a second

  digitalWrite(LEDG, LOW);   // turn the LED on
  delay(1000);               // wait for a second

  digitalWrite(LEDB, LOW);   // turn the LED on
  delay(1000);               // wait for a second
}
*/


#include <LSM6DS3.h>  // accelerometer, gyroscope
//Create a instance of class LSM6DS3
//LSM6DS3 pedometer(I2C_MODE, 0x6A);    //I2C device address 0x6A
LSM6DS3 gSensor(I2C_MODE, LSM6DS3_C_ACC_GYRO_WHO_AM_I);
#define int1Pin PIN_LSM6DS3TR_C_INT1
uint8_t tapCount = 0; // Amount of received interrupts
uint8_t prevTapCount = 0; // Interrupt Counter from last loop
unsigned long lastTouchTime = 0;

#define CLEAR_STEP      true
#define NOT_CLEAR_STEP  false
uint16_t gStepCount = 0;

#include <Adafruit_NeoPixel.h>  // WS2812 led strip

#define PIN        0
#define NUMPIXELS 24

Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
uint16_t current_hue = 0;

// https://wiki.seeedstudio.com/XIAO-BLE-Sense-Bluetooth-Usage/
#include <ArduinoBLE.h>
 
BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service
 
// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic brightnessCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic saturationCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const byte manufacturerData[] = { 0x54, 0x6f, 0x75, 0x63, 0x68, 0x42, 0 };
const int ledPin = LED_BUILTIN; // pin to use for the LED

unsigned long myTime = 0;

#define Serial_println if (Serial) Serial.println
#define Serial_print if (Serial) Serial.print

void setup() {
  Serial.begin(9600);
  //while (!Serial);
  delay(1000);
 
  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);
  pinMode(LEDB, OUTPUT);
 
  // begin initialization
  if (!BLE.begin()) {
      Serial_println(F("starting Bluetooth® Low Energy module failed!"));
 
    while (1);
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
  
  Serial_println(F("BLE LED Peripheral"));

  strip.begin();
  strip.show();
  strip.setBrightness(0xff);

  gSensor.settings.gyroEnabled = 0; // Gyro currently not used, disabled to save power 
  if (gSensor.begin() != 0) {
      Serial_println(F("Device error"));
  } else {
      Serial_println(F("Device OK!"));
  }
  //Configure LSM6DS3 as pedometer
  if (0 != config_pedometer(NOT_CLEAR_STEP)) {
      Serial_println(F("Configure pedometer fail!"));
  }
  //Configure LSM6DS3 as touch sensor
  if (0 != config_touch_detect()) {
      Serial_println(F("Configure touch detect fail!"));
  }
}
 
void loop() {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();
 
  // if a central is connected to peripheral:
  if (central) {
    digitalWrite(LEDB, HIGH);  // turn the LED off
    Serial_print(F("Connected to central: "));
    // print the central's MAC address:
    Serial_println(central.address());
 
    // while the central is still connected to peripheral:
    while (central.connected()) {
        if (switchCharacteristic.written()) {
          if (switchCharacteristic.value()) {   
            Serial_println(F("LED on"));
            digitalWrite(ledPin, LOW); // changed from HIGH to LOW       
          } else {                       
            Serial_println(F("LED off"));
            digitalWrite(ledPin, HIGH); // changed from LOW to HIGH     
          }
        }
        updateLedStrip();
        updateTapAction();
    }

    // when the central disconnects, print it out:
    Serial_print(F("Disconnected from central: "));
    Serial_println(central.address());
  } else {
    // show blue when not connected
    digitalWrite(LEDB, LOW);   // turn the LED on
  }

  updateTapAction();
  updatePedometer();
  updateLedStrip();
  updateTimer();
}

void updateTimer() {
    unsigned long now = millis();
    unsigned long delta = now - myTime;
//      if (Serial) {
//        Serial.print(F("DT="));
//        Serial.println(delta);
//      }
    if (delta > 1000) {
      myTime = now;
      Serial_println(BLE.address());
    }
}

void updateLedStrip() {
  current_hue = current_hue + 43;
  strip.rainbow(current_hue, 1, saturationCharacteristic.value(), brightnessCharacteristic.value());
  strip.show();
}

void updatePedometer() {
//    uint8_t dataByte = 0;
//    uint16_t stepCount = 0;
//
//    pedometer.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
//    stepCount = (dataByte << 8) & 0xFFFF;
//
//    pedometer.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
//    stepCount |=  dataByte;
//
//    if (gStepCount != stepCount) {
//      gStepCount = stepCount;
//      Serial_print("Step: ");
//      Serial_println(stepCount);
//    }
}

void updateTapAction() {
  unsigned long now = millis();
  if ((now - lastTouchTime) > 100)
    digitalWrite(LEDG, HIGH);   // turn green LED off
  
  int delta = tapCount - prevTapCount;
  if (delta == 0)
    return;
  prevTapCount += delta;
  lastTouchTime = now;
    
  Serial_print(F("Tap detected "));
  Serial_println(delta);

  digitalWrite(LEDG, LOW);   // turn green LED on
}

//Setup pedometer mode
int config_pedometer(bool clearStep) {
    uint8_t errorAccumulator = 0;
//    uint8_t dataToWrite = 0;  //Temporary variable
//
//    //Setup the accelerometer******************************
//    dataToWrite = 0;
//
//    //  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
//    dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
//    dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
//
//
//    // Step 1: Configure ODR-26Hz and FS-2g
//    errorAccumulator += pedometer.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);
//
//    // Step 2: Set bit Zen_G, Yen_G, Xen_G, FUNC_EN, PEDO_RST_STEP(1 or 0)
//    if (clearStep) {
//        errorAccumulator += pedometer.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3E);
//    } else {
//        errorAccumulator += pedometer.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3C);
//    }
//
//    // Step 3:  Enable pedometer algorithm
//    errorAccumulator += pedometer.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);
//
//    //Step 4: Step Detector interrupt driven to INT1 pin, set bit INT1_FIFO_OVR
//    errorAccumulator += pedometer.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x10);
//
    return errorAccumulator;
}

// taken from https://forum.seeedstudio.com/t/xiao-ble-sense-lsm6ds3-int1-single-tap-interrupt/264206/6
int config_touch_detect() {
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

void int1ISR()
{
  tapCount++;
}
