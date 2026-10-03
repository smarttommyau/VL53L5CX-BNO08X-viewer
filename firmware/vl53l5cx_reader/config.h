#define I2C_SPEED 1000000 
// #define I2C_SPEED 400000  // 400kHz is more stable for continuous streaming


// Version - must match viewer config.VERSION
#define VERSION "0.1.0-ust"

// WiFi Mode if defined, wifi will be used to send data to the viewer, otherwise USB will be used
// #define WIFI_MODE
// #define WIFI_SSID "your_wifi_ssid"
// #define WIFI_PASSWORD "your_wifi_password"

// BLE Mode if defined, BLE will be used to send data to the viewer, otherwise USB will be used
// #define BLE_MODE


// Serial baud rate for USB mode
#define SERIAL_BAUD 115200


// if define we use 2 sensor
// #define MULTIPLE_SENSORS

// for sesnor 1
// Pin definitions
#define SDA_PIN 4
#define SCL_PIN 5

// for sesnor 2
#define SDA_PIN_2 17
#define SCL_PIN_2 18