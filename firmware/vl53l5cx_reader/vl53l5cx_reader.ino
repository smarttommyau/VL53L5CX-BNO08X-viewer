/*
 * VL53L5CX ToF Sensor Reader for ESP32
 *
 * Reads 8x8 distance data from VL53L5CX and outputs JSON over serial.
 *
 * Wiring (both sensors share I2C bus):
 *   VIN -> 3V3
 *   GND -> GND
 *   SDA -> GPIO 4
 *   SCL -> GPIO 5
 */

#include "config.h"
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "vl53l5cx_features.h"
#include "connection_adapter.h"

// Modified to suit our layout



// sesnors
VL53L5CXSensor* sensor0 = 0;
#ifdef MULTIPLE_SENSORS
VL53L5CXSensor* sensor1 = 0;
#endif

void setup() {
  // Initialize serial communication
  ConnectionAdapter::getInstance().setup();
  // Initialize the VL53L5CX sensor
  sensor0 = new VL53L5CXSensor(SDA_PIN, SCL_PIN);
  #ifdef MULTIPLE_SENSORS
  sensor1 = new VL53L5CXSensor(SDA_PIN_2, SCL_PIN_2,&Wire1);
  #endif
}

void loop() {

  // Check if new ToF data is available
  if (sensor0)
    sensor0->check();
  
  #ifdef MULTIPLE_SENSORS
  if (sensor1)
    sensor1->check();
  #endif

  #ifdef WIFI_MODE
  ConnectionAdapter::getInstance().loop();
  #endif

  // Small delay to prevent overwhelming the serial buffer
  delay(1);
}
