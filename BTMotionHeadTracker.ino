/*
 * Sets BLE characteristic options
 * Also shows how to set a custom MAC address
 * Use BLE Scanner etc on Android to see them
 */

#include <Arduino.h>
#include <BleGamepad.h>
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps612.h>
#include <Wire.h>

// I2C data
#define PIN_SDA 23
#define PIN_SCL 19
#define I2C_400kHz 400000

int16_t gamepadAxesMin = 0x0000;
int16_t gamepadAxesMax = 0x7FFF;
BleGamepad bleGamepad("Motion HeadTracker", "Shenron", 100); // Set custom device name, manufacturer and initial battery level

// Use the procedure below to set a custom Bluetooth MAC address
// Compiler adds 0x02 to the last value of board's base MAC address to get the BT MAC address, so take 0x02 away from the value you actually want when setting
// I've noticed the first number is a little picky and if set incorrectly don't work and will default to the board's embedded address
// 0xAA definately works, so use that, or experiment
//uint8_t MACAddress[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF - 0x02};

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
uint16_t dmpPacketSize;
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q; // quaternion container [w, x, y, z]
float gyro[3]; // Gyro angles container
float radToDegre = 180 / M_PI; // to convert radian angles to degre
float gyroXdelta = 15.0, gyroYdelta = -15.0, gyroXscale, gyroYscale;
long delayUpdate = 50; // millis between updates
long nextUpdate = 0;

void setup()
{
  delay(500);
  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz)
  Serial.begin(38400);
  do { delay(10); } while(Serial == false);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_400kHz);

  InitializeMpu();
  InitializeBleGamepad();

  float axesAmplitude = gamepadAxesMax - gamepadAxesMin;
  gyroXscale = axesAmplitude / (gyroXdelta * 2.0);
  gyroYscale = axesAmplitude / (gyroYdelta * 2.0);
  // Set new MAC address
  //esp_base_mac_addr_set(&MACAddress[0]);

  delay(3000);
}

void loop()
{
  long timeNow = millis();
  if(timeNow >= nextUpdate)
  {
    ReadMpu(false);    
    UpdateGamepadAxis(true);
    nextUpdate = timeNow + delayUpdate;
  }
}

void InitializeMpu()
{
  // initialize device
  Serial.println("Initializing MPU...");
  mpu.initialize();
  Serial.println("Initializing DMP...");
  mpu.dmpInitialize();
  Serial.println("Generating offsets and calibrating MPU...");
  mpu.CalibrateGyro();
  Serial.print("Active offsets: ");
  mpu.PrintActiveOffsets();
  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  // get expected DMP packet size for later comparison
  dmpPacketSize = mpu.dmpGetFIFOPacketSize();
}

void InitializeBleGamepad()
{
  Serial.println("Configuring BleGamepad...");
  BleGamepadConfiguration bleGamepadConfig; // Create a BleGamepadConfiguration object to store all of the options
  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD); // CONTROLLER_TYPE_JOYSTICK, CONTROLLER_TYPE_GAMEPAD (DEFAULT), CONTROLLER_TYPE_MULTI_AXIS
  bleGamepadConfig.setVid(0xe502);
  bleGamepadConfig.setPid(0xabcd);
  bleGamepadConfig.setModelNumber("1.0");
  bleGamepadConfig.setSoftwareRevision("Software Rev 1");
  bleGamepadConfig.setSerialNumber("20230106");
  bleGamepadConfig.setFirmwareRevision("1.0");
  bleGamepadConfig.setHardwareRevision("1.0");
  // Gamepad characeristics
  bleGamepadConfig.setButtonCount(1);
  bleGamepadConfig.setIncludeXAxis(true);
  bleGamepadConfig.setIncludeYAxis(true);
  // Some non-Windows operating systems and web based gamepad testers don't like min axis set below 0, so 0 is set by default
  bleGamepadConfig.setAxesMin(gamepadAxesMin); // 0 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
  bleGamepadConfig.setAxesMax(gamepadAxesMax); // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
  bleGamepad.begin(&bleGamepadConfig); // Begin gamepad with configuration options
  Serial.print("BleGamepad initialized with axes min=[");
  Serial.print(gamepadAxesMin);
  Serial.print("] and max=[");
  Serial.print(gamepadAxesMax);
  Serial.println("]");
}

void ReadMpu(bool echoSerial)
{
  // read the Latest packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer) == false)
    return;

  // quaternion values in easy matrix form: w x y z
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  // Get euler angles
  mpu.dmpGetEuler(gyro, &q);
  for(int index=0; index < 3; ++index)
    gyro[index] *= radToDegre;

  if(echoSerial == false)
    return;

  // display on serial
  Serial.printf("gyro:\tx: %3.3f\ty: %3.3f\tz: %3.3f", gyro[0], gyro[1], gyro[2]);
  Serial.println();
}

void UpdateGamepadAxis(bool echoSerial)
{
  if (bleGamepad.isConnected() == false)
    return;

  int16_t axeX = getAxe(gyro[0], gyroXdelta, gyroXscale);
  int16_t axeY = getAxe(gyro[1], gyroYdelta, gyroYscale);

  bleGamepad.setX(axeX);
  bleGamepad.setY(axeY);
  bleGamepad.sendReport();

  if(echoSerial == false)
    return;

  Serial.printf("bleGamepad axeX: %6d\taxeY: %6d", axeX, axeY);
  Serial.println();
}

int16_t getAxe(float value, float delta, float scale)
{
  int result = (int)((value + delta) * scale);
  if(result < gamepadAxesMin) return gamepadAxesMin;
  if(result > gamepadAxesMax) return gamepadAxesMax;
  return result;
}
