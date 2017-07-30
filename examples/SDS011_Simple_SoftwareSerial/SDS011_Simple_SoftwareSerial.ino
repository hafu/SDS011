#include "SDS011.h"

#include <SoftwareSerial.h>

// pins for SoftwareSerial
#define SDS_PIN_RX D5
#define SDS_PIN_TX D6

// Setup SoftwareSerial and Sensor
SoftwareSerial serialSDS(SDS_PIN_RX, SDS_PIN_TX, false, 128);
SDS011 sds011(serialSDS);

void setup() {
    Serial.begin(115200);

    // default rate for sensor
    serialSDS.begin(9600);

}

void loop() {
    float pm25, pm10;

    // read sensor, print readings, irgnore errors
    if (sds011.getData(&pm25, &pm10) == SDS011_STATUS_OK) {
        Serial.print("PM2.5: "); Serial.println(pm25, 2);
        Serial.print("PM10:  "); Serial.println(pm10, 2);
        Serial.print(pm25, 1); Serial.print("\t"); Serial.println(pm10, 2);
    }

    delay(100);
}
