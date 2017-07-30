#include "SDS011.h"

#include <SoftwareSerial.h>

// pins for SoftwareSerial
#define SDS_PIN_RX D5
#define SDS_PIN_TX D6

// how often should be measuered
#define MEASUREMENT_INTERVAL    60000
// time before reading the sensor (give the fan some spin up time :)
#define WARMUP_TIME             5000
// how much samples should be taken
#define MEASUREMENT_SAMPLES     15


// Setup SoftwareSerial and Sensor
SoftwareSerial serialSDS(SDS_PIN_RX, SDS_PIN_TX, false, 128);
SDS011 sds011(serialSDS);

// some vars to hold time, sample count and the samples
unsigned long t_last_measurement = 0;
uint8_t sample_count = 0;
float pm25_samples[MEASUREMENT_SAMPLES];
float pm10_samples[MEASUREMENT_SAMPLES];

void setup() {
    Serial.begin(115200);
    // default rate for sensor
    serialSDS.begin(9600);
    
    // set it to sleep
    sds011.sleep();
}

void loop() {
    // activate sensor (MEASUREMENT_INTERVAL - WARMUP_TIME)
    if (!sds011.isActive() && (millis() - t_last_measurement >= MEASUREMENT_INTERVAL - WARMUP_TIME)) {
        Serial.println("Activate SDS to warmup");
        sds011.activate();
    }

    // get the samples and put them into the arrays
    if (sds011.isActive() && (millis() - t_last_measurement >= MEASUREMENT_INTERVAL)) {
        uint8_t sds_staus;
        Serial.print("Do measurement: ");
        switch (sds011.getData(&pm25_samples[sample_count], &pm10_samples[sample_count])) {
            case SDS011_STATUS_OK:
                Serial.print("PM2.5: "); Serial.print(pm25_samples[sample_count], 2);
                Serial.print("\tPM10: "); Serial.println(pm10_samples[sample_count], 2);
                sample_count++;
                break;
            case SDS011_STATUS_TIMEOUT:
                Serial.println("A timeout occured");
                break;
            case SDS011_STATUS_READ_ERROR:
                Serial.println("Read error after header byte");
                break;
            case SDS011_STATUS_CRC_ERROR:
                Serial.println("CRC sum did not match");
                break;
            default:
                Serial.println("Undefined status code, this should not happen");
                break;
        }
    }

    // enough samples, calculate avg values and set sensor to sleep
    if (sds011.isActive() && sample_count >= MEASUREMENT_SAMPLES) {
        float pm25_avg, pm10_avg;
        sds011.filterData(&pm25_avg, &pm10_avg, &pm25_samples[0], &pm10_samples[0], MEASUREMENT_SAMPLES);
        Serial.print("AVG values:     "); Serial.print("PM2.5: "); Serial.print(pm25_avg);
        Serial.print("\tPM10: "); Serial.println(pm10_avg);
        Serial.println("Deactivate SDS to sleep");
        // reset sample counter and sample arrays
        sample_count = 0;
        memset(pm25_samples, 0x00, sizeof(pm25_samples));
        memset(pm10_samples, 0x00, sizeof(pm10_samples));
        sds011.sleep();
        t_last_measurement = millis();
    }
    
    delay(100);
}
