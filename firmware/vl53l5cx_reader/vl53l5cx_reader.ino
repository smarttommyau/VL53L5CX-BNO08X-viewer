/*
 * VL53L5CX ToF Sensor + BNO08X IMU Reader for ESP32
 *
 * Reads 8x8 distance data from VL53L5CX and orientation from BNO08X,
 * outputs JSON over serial.
 *
 * Wiring (both sensors share I2C bus):
 *   VIN -> 3V3
 *   GND -> GND
 *   SDA -> GPIO 4
 *   SCL -> GPIO 5
 */

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "vl53l5cx_features.h"


// Pin definitions
#define SDA_PIN 4
#define SCL_PIN 5
// Modified to suit our layout


// sesnors
VL53L5CXSensor* sensor0 = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB
  }

  // Initialize the VL53L5CX sensor
  sensor0 = new VL53L5CXSensor(SDA_PIN, SCL_PIN);


}

void loop() {

  // Check if new ToF data is available
  if (sensor0)
    sensor0->check();
  
  // Small delay to prevent overwhelming the serial buffer
  delay(1);
}
