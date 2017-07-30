/**
 * @file SDS011.cpp
 * @author Hannes Fuchs
 * @date 2017-07-30
 * @copyright MIT 
 * @brief Main Class file for SDS011 Sensor library
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

#include "SDS011.h"


/**
 * @breif The main class for the SDS011 sensor library
 *
 * Sets:
 *  - the stream (could be Serial/SoftwareSerial)
 *  - default stream timeout
 *  - default active status
 */
SDS011::SDS011(Stream &s) : _s(s) {
    // set a safe timeout 
    _s.setTimeout(SDS011_READ_TIME*2);
    // if powerd on it is in "active" reporting mode
    _is_active = true;
}

/**
 * @brief Reads the current reported values from the Sensor
 *
 * Waits for "magic" header byte message on stream until the "magic" tail
 * byte or until the rest of the _data buffer is full. Afterwards the checksum 
 * will be calculated. Only if the communication was successful and the 
 * checksum is correct the provided params will be set.
 *
 * @param pointer to vars to hold the readed values
 * @return SDS011_STATUS_OK | SDS011_STATUS_READ_ERROR | SDS011_STATUS_CRC_ERROR
 */
uint8_t SDS011::getData(float *pm25, float *pm10) {
    uint8_t status;
    uint8_t crc = 0;

    // unset data buffer
    memset(_data, 0x00, sizeof(_data));

    // wait for header byte (SDS011_MSG_HEADER)
    while (_data[SDS011_OFFSET_MSG_HEADER] != SDS011_MSG_HEADER) {
        if ((status = _readByte(&_data[SDS011_OFFSET_MSG_HEADER])) 
                != SDS011_STATUS_OK) {
            return status;
        }
    }

    // now read the next bytes until SDS011_MSG_TAIL or _data size
    if (_s.readBytesUntil(SDS011_MSG_TAIL, &_data[SDS011_OFFSET_CMD_NO], 
            (sizeof(_data)/sizeof(_data[0]))-SDS011_OFFSET_CMD_NO) == 0) {
        return SDS011_STATUS_READ_ERROR;
    }
    // do not set the SDS011_MSG_TAIL in _data buffer, it is not needed
    
    // checking the crc
    for (uint8_t i=SDS011_OFFSET_PM25_LB; i<SDS011_OFFSET_CHK_SUM; i++) {
        crc += _data[i];
    }
    if (crc != _data[SDS011_OFFSET_CHK_SUM]) {
        SDS011_DEBUG_PRINTLN(F("CRC Error"));
        return SDS011_STATUS_CRC_ERROR;
    }

    // now set set the values, see p.6
    *pm25 = float(uint16_t(_data[SDS011_OFFSET_PM25_LB] 
                | _data[SDS011_OFFSET_PM25_HB]<<8) / 10.0);
    *pm10 = float(uint16_t(_data[SDS011_OFFSET_PM10_LB] 
                | _data[SDS011_OFFSET_PM10_HB]<<8) / 10.0);


    // flush the buffer
    _s.flush();
    return status;
}

/**
 * @brief Sets the sensor into sleep mode
 *
 * Fan and laser should be deactivated and the sensor will not report anything
 * until it is activated again. Should increase the lifetime of the sensor.
 *
 * NOTE:
 *  I did not found any documentation of this command, only some code which use
 *  this. 
 *
 * @see https://github.com/opendata-stuttgart/sensors-software/blob/44f8d7040847cc982d16b3ad9aab6d839b08fd01/airrohr-firmware/airrohr-firmware.ino#L483
 */
void SDS011::sleep(void) {
    uint8_t cmd[] = {0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB};
    _sendBytes(cmd, sizeof(cmd));
    _is_active = false;
}

/**
 * @brief Sets the sensor into active mode
 *
 * Recover from sleep mode and activate the sensor again. Default behaviour 
 * after power on.
 *
 * NOTE:
 *  I did not found any documentation of this command, only some code which use
 *  this. 
 *
 * @see https://github.com/opendata-stuttgart/sensors-software/blob/44f8d7040847cc982d16b3ad9aab6d839b08fd01/airrohr-firmware/airrohr-firmware.ino#L475
 */
void SDS011::activate(void) {
    uint8_t cmd[] = {0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB};
    _sendBytes(cmd, sizeof(cmd));
    _is_active = true;
}

/**
 * @bref Reads one byte from the stream
 *
 * Reads exactly ony byte from stream. Waits for availability of stream and can
 * run into a timeout.
 *
 * @param pointer to byte to write to
 * @return SDS011_STATUS_OK | SDS011_STATUS_TIMEOUT
 */
uint8_t SDS011::_readByte(uint8_t *b) {
    uint16_t t = 0;

    // wait for stream
    while (!_s.available()) {
        if (t > SDS011_READ_TIME) {
            SDS011_DEBUG_PRINTLN(F("A timeout occured"));
            return SDS011_STATUS_TIMEOUT;
        }
        t++;
        delay(1);
    }

    // read it
    *b = (uint8_t)_s.read();
    return SDS011_STATUS_OK;
}

/**
 * @brief Send bytes to stream
 *
 * Just a wrapper for the write() function. After write the stream will be 
 * flushed.
 *
 * TODO
 *  - Check return values of write() and flush()?
 *
 *  @params buffer and buffer length
 */
void SDS011::_sendBytes(uint8_t *b, uint8_t len) {
    _s.write(b, len);
    _s.flush();
}

/**
 * @brief Removes the highest and lowest values from a series of data and 
 *        calculates the avarage value
 *
 * If there are more than two items, the avarage values will be calculated. In
 * the calculation the lowest and highest values will be excluded to get more
 * accurate data. So on 3 values only the mid one will be used.
 * On a value count <= 2 no value will be removed and therefore the avarage is
 * calculated of all values.
 *
 * NOTE: src arrays should have the same size!
 *
 * @param dst pm25 (avg), dst pm10 (avg), src pm25 (arr), src pm10 (arr), 
 *        items in array  
 */
void SDS011::filterData(float *pm25_avg, float *pm10_avg, 
        float *pm25_arr, float *pm10_arr, uint8_t items) {
    float pm25_min, pm25_max, pm25_sum, pm10_min, pm10_max, pm10_sum;
    
    // set inital values
    pm25_min = pm25_max = pm25_sum = pm25_arr[0];
    pm10_min = pm10_max = pm10_sum = pm10_arr[0];
    // find min/max values and calculate sum
    for (uint8_t i=1; i<items; i++) {
        pm25_min = min(pm25_arr[i], pm25_min);
        pm25_max = max(pm25_arr[i], pm25_max);
        pm10_min = min(pm10_arr[i], pm10_min);
        pm10_max = max(pm10_arr[i], pm10_max);

        pm25_sum += pm25_arr[i];
        pm10_sum += pm10_arr[i];
    }

    // more than 3 items -> remove min/max
    if (items > 2) {
        *pm25_avg = (pm25_sum - pm25_min - pm25_max) / (items - 2);
        *pm10_avg = (pm10_sum - pm10_min - pm10_max) / (items - 2);
    } else {
        *pm25_avg = pm25_sum / items;
        *pm10_avg = pm10_sum / items;
    }

}

/**
 * @brief Wrapper for _is_active value
 *
 * @return Status of the sensor (active|sleeping)
 */
bool SDS011::isActive(void) {
    return _is_active;
}
