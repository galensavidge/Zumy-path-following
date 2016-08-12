/*
 * An mbed library for the TSL1401CL line sensor.
 * Author: Galen Savidge
 */

#ifndef TSL1401CLh
#define TSL1401CLh

#include "mbed.h"

#define TSL1401CL_PIXEL_COUNT 128

class TSL1401CL
{
public:
    TSL1401CL(PinName pin_clk, PinName pin_si, PinName pin_adc);
    
    /**
    * Returns whether enough integration time has passsed for the sensor to be ready to read.
    */
    bool integrationReady();
    
    /**
    * Reads from the sensor into private data storage.
    */
    void read();
    
    /**
    * Sets the integration time of the sensor. Can be called mid integration.
    */
    void setIntegrationTime(uint32_t int_time_desired);

    /**
    * Returns the dark-to-light edge of a line as an int between 0 and cam resolution. Returns
    * -1 when no line is found. Use higher precision for fewer samples, default is 1 (lowest).
    * Ignores crop_amount pixels on both sides of data. Set invert to true to instead return the
    * light-to-dark line edge (if found).
    */
    int findLineEdge (uint16_t threshold, uint8_t precision = 1, size_t crop_amount = 0, bool invert = false);
    
    /**
    * Like FindLineEdge, but finds both edges of the line and returns their midpoint. Returns -1
    * if either edge is not found.
    */
    int findLineCenter (uint16_t threshold, uint8_t precision = 1, size_t crop_amount = 0);
    
    /**
    * Returns the value of the sensor's last read data at a specified index.
    */
    uint32_t getData(uint8_t index);
protected:
    DigitalOut clk;
    DigitalOut si;
    AnalogIn adc;

    uint32_t data[TSL1401CL_PIXEL_COUNT];
    uint32_t integration_time;
    Timer integration_timer;
};

#endif