/**
 * @file SDS011.h
 * @author Hannes Fuchs
 * @date 2017-07-30
 * @copyright MIT 
 * @brief Header file for SDS011 Sensor library
 *
 * This is a library for the Nova SDS011 dust sensor. More informations can be 
 * found under http://inovafitness.com/en/a/chanpinzhongxin/95.html
 * Sadly the documentation of the UART communication in the provided datasheet
 * is very limited. Most logic comes from other libraries. [1], [2]
 *
 * TODO (Need UART communication documentation)
 *  - parse return message of sleep() and activate()
 *  - switch modes, active (default), query and even more? 
 *  - read out FW version
 * 
 * [1] https://github.com/kadamski/arduino_sds011/tree/9669fda1ab3f47b4b3c8ec504609c05ad131ff9f/lib/Sds011
 * [2] https://github.com/opendata-stuttgart/sensors-software/blob/44f8d7040847cc982d16b3ad9aab6d839b08fd01/airrohr-firmware/airrohr-firmware.ino
 *
 * @see http://inovafitness.com/en/a/chanpinzhongxin/95.html
 */

#ifndef _SDS011_H
#define _SDS011_H

#if ARDUINO >= 100
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif

#include <Stream.h>


// uncomment to enable debugging output
//#define SDS011_DEBUG

// output for debugging output
#define SDS011_DEBUG_PRINTER Serial

// debug macros
#ifdef SDS011_DEBUG
    #define SDS011_DEBUG_PRINT(...) { SDS011_DEBUG_PRINTER.print(__VA_ARGS__); }
    #define SDS011_DEBUG_PRINTLN(...) { SDS011_DEBUG_PRINTER.println(__VA_ARGS__); }
#else
    #define SDS011_DEBUG_PRINT(...) {}
    #define SDS011_DEBUG_PRINTLN(...) {}
#endif

#define SDS011_MSG_HEADER			(0xAA)
#define SDS011_MSG_TAIL				(0xAB)

// adressing
#define SDS011_OFFSET_MSG_HEADER	(0x00)
#define SDS011_OFFSET_CMD_NO		(0x01)
#define SDS011_OFFSET_PM25_LB		(0x02)
#define SDS011_OFFSET_PM25_HB		(0x03)
#define SDS011_OFFSET_PM10_LB		(0x04)
#define SDS011_OFFSET_PM10_HB		(0x05)
#define SDS011_OFFSET_ID_B1			(0x06)
#define SDS011_OFFSET_ID_B2			(0x07)
#define SDS011_OFFSET_CHK_SUM		(0x08)
#define SDS011_OFFSET_MSG_TAIL		(0x09)

// status codes
#define SDS011_STATUS_OK            (0x00)
#define SDS011_STATUS_TIMEOUT       (0x01)
#define SDS011_STATUS_READ_ERROR    (0x02)
#define SDS011_STATUS_CRC_ERROR     (0x03)

// reading should need max 1015ms (see datasheet p.6)
#define SDS011_READ_TIME            (1015)

class SDS011 {
    public:
        SDS011(Stream &s);
        uint8_t getData(float *pm25, float *pm10);
        void filterData(float *pm25_avg, float *pm10_avg, 
                float *pm25_arr, float *pm10_arr, uint8_t items);
        void sleep(void);
        void activate(void);
        bool isActive(void);
    private:
        uint8_t _readByte(uint8_t *b);
        void _sendBytes(uint8_t *b, uint8_t len);

        uint8_t _data[10];
        Stream &_s;
        bool _is_active;
};

#endif /* _SDS011_H */
