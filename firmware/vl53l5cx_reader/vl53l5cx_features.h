#ifndef VLL53LCX_FEATURES_H
#define VLL53LCX_FEATURES_H
#include <SparkFun_VL53L5CX_Library.h>
#include "config.h"

class VL53L5CXSensor {
    SparkFun_VL53L5CX sensor;
    VL53L5CX_ResultsData measurementData;
    TwoWire *wirePort;

    // pins
    int sdaPin;
    int sclPin;

    public:
        VL53L5CXSensor(int sda, int scl, TwoWire *wire = &Wire) : sdaPin(sda), sclPin(scl), wirePort(wire) {

            wirePort->begin(sdaPin, sclPin);
            wirePort->setClock(I2C_SPEED);

            Serial.printf("{\"status\":\"i2c_ready\", \"sda\":%d}\n", sdaPin);

            // Initialize sensor
            if (!sensor.begin(DEFAULT_I2C_ADDR >> 1, *wirePort)) {
                Serial.printf("{\"error\":\"sensor_init_failed\", \"sda\":%d}\n", sdaPin);
                while (1) {
                delay(1000);
                }
            }
            Serial.printf("{\"status\":\"sensor_found\", \"sda\":%d}\n", sdaPin);

             // Configure sensor for 8x8 resolution
            sensor.setResolution(64);  // 64 zones = 8x8

            // Set ranging frequency to 15Hz (stable for continuous streaming)
            sensor.setRangingFrequency(15);

            // Start ranging
            sensor.startRanging();

            Serial.printf("{\"status\":\"ranging_started\",\"resolution\":\"8x8\",\"frequency_hz\":15, \"sda\":%d}\n", sdaPin);
        }

        void check() {
              // Check if new ToF data is available
            if (sensor.isDataReady()) {
                if (sensor.getRangingData(&measurementData)) {
                // Output JSON with distance and status data
                Serial.print("{\"distances\":[");

                for (int i = 0; i < 64; i++) {
                    // Distance in mm
                    Serial.print(measurementData.distance_mm[i]);
                    if (i < 63) Serial.print(",");
                }

                Serial.print("],\"status\":[");

                for (int i = 0; i < 64; i++) {
                    // Target status (5 = valid, others = various error states)
                    Serial.print(measurementData.target_status[i]);
                    if (i < 63) Serial.print(",");
                }
                Serial.print("],\"v\":\"");
                Serial.print(VERSION);
                Serial.print("\",\"sda\":");
                Serial.print(sdaPin);
                Serial.println("}");
                }
            }
        }


};

#endif