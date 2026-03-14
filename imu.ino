// BNO055 IMU - wiring:
//   VIN → 5V, GND → GND, SDA → SDA (pin 20), SCL → SCL (pin 21)
//   Shares I2C bus with ToF sensors (no address conflict: 0x28)
//
// Requires: Adafruit BNO055 library + Adafruit Unified Sensor library

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Wire.begin() is called in main.ino before setupImu().
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setupImu()
{
  while (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring.");
    // while (1);
  }
  bno.setExtCrystalUse(true);

  // Debug: Check IMU calibration
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print("Cal: sys="); Serial.print(sys);
  Serial.print(" gyro="); Serial.print(gyro);
  Serial.print(" mag="); Serial.println(mag);
  // Delay for gyro calibration/Users/bealim/Library/Mobile Documents/com~apple~CloudDocs/Stanny/ME210/final_project/me210/final/main/main.ino
  delay(1000);
  // Debug: Check IMU calibration
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print("Cal: sys="); Serial.print(sys);
  Serial.print(" gyro="); Serial.print(gyro);
  Serial.print(" mag="); Serial.println(mag);
}

// Fills heading (yaw), pitch, roll in degrees. Heading 0-360, pitch/roll +-180.
void pollImu(float &heading, float &pitch, float &roll)
{
  sensors_event_t event;
  bno.getEvent(&event);
  heading = event.orientation.x;
  pitch   = event.orientation.y;
  roll    = event.orientation.z;
  // Serial.print("heading: ");
  // Serial.print(heading);
  // Serial.println();
}
