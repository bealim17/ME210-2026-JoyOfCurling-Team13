// Two VL53L4CD sensors sharing one I2C bus on Arduino Mega.
// Both sensors share SDA/SCL. Each needs its own XSHUT pin.
// Wiring:
//   Sensor 1: VCC->3.3V, GND->GND, SDA->SDA, SCL->SCL, XSHUT->A1
//   Sensor 2: VCC->3.3V, GND->GND, SDA->SDA, SCL->SCL, XSHUT->A2
//
// Both sensors boot at the same default address (0x52). We use XSHUT to
// disable sensor 2 while we reassign sensor 1 to 0x54, then enable sensor 2
// so it keeps the default 0x52. After setup both run concurrently on the bus.

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <stdint.h>

#define DEV_I2C Wire
#define SerialPort Serial

#define PIN_XSHUT_1 53
#define PIN_XSHUT_2 49

#define TOF1_ADDR 0x54  // reassigned address for sensor 1
#define TOF2_ADDR 0x52  // default address kept for sensor 2

VL53L4CD sensor1(&DEV_I2C, PIN_XSHUT_1);
VL53L4CD sensor2(&DEV_I2C, PIN_XSHUT_2);

void setupTof()
{
  // Hold both sensors in reset so neither is on the bus yet.
  pinMode(PIN_XSHUT_1, OUTPUT);
  pinMode(PIN_XSHUT_2, OUTPUT);
  digitalWrite(PIN_XSHUT_1, LOW);
  digitalWrite(PIN_XSHUT_2, LOW);
  delay(10);

  SerialPort.println("tof init1");

  // Boot sensor 1 alone and give it a new address.
  digitalWrite(PIN_XSHUT_1, HIGH);
  delay(100);
  SerialPort.println("tof init11");
  sensor1.begin();
  SerialPort.println("tof init12");
  sensor1.VL53L4CD_Off();
  SerialPort.println("tof init13");
  sensor1.InitSensor(TOF1_ADDR);
  SerialPort.println("tof init2");

  // Now boot sensor 2; it comes up at the default address (0x52).
  digitalWrite(PIN_XSHUT_2, HIGH);
  delay(10);
  sensor2.begin();
  sensor2.VL53L4CD_Off();
  sensor2.InitSensor(TOF2_ADDR);
  SerialPort.println("tof init3");

  sensor1.VL53L4CD_SetRangeTiming(200, 0);
  sensor2.VL53L4CD_SetRangeTiming(200, 0);
  SerialPort.println("tof init4");

  sensor1.VL53L4CD_StartRanging();
  sensor2.VL53L4CD_StartRanging();

  SerialPort.println("tof init5");
}

// Returns distance in mm from sensor 1, or -1 if not ready / error.
int readTof1()
{
  uint8_t ready = 0;
  VL53L4CD_Result_t result;
  sensor1.VL53L4CD_CheckForDataReady(&ready);
  if (!ready) return -1;
  sensor1.VL53L4CD_ClearInterrupt();
  sensor1.VL53L4CD_GetResult(&result);
  return (result.range_status == 0) ? result.distance_mm : -1;
}

// Returns distance in mm from sensor 2, or -1 if not ready / error.
int readTof2()
{
  uint8_t ready = 0;
  VL53L4CD_Result_t result;
  sensor2.VL53L4CD_CheckForDataReady(&ready);
  if (!ready) return -1;
  sensor2.VL53L4CD_ClearInterrupt();
  sensor2.VL53L4CD_GetResult(&result);
  return (result.range_status == 0) ? result.distance_mm : -1;
}

void pollTof(int &d1, int &d2)
{
  d1 = readTof1() /10 ; // Right
  d2 = readTof2() / 10; // Left
  // SerialPort.print("ToF: "), SerialPort.print(d1), SerialPort.print("cm\t"), SerialPort.print(d2), SerialPort.println("cm");
  // if (d1 >= 0) SerialPort.print("Sensor1: "), SerialPort.print(d1), SerialPort.println(" mm");
  // if (d2 >= 0) SerialPort.print("Sensor2: "), SerialPort.print(d2), SerialPort.println(" mm");
}