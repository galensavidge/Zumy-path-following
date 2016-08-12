#include "TSL1401CL.h"

TSL1401CL::TSL1401CL(PinName pin_clk, PinName pin_si, PinName pin_adc) : clk(pin_clk), si(pin_si), adc(pin_adc), integration_time(0)
{
    clk.write(0);
    integration_timer.start();
}

bool TSL1401CL::integrationReady()
{
    return static_cast<uint32_t>(integration_timer.read_us()) >= integration_time;
}

void TSL1401CL::read()
{
    integration_timer.reset();
    integration_timer.start();
    clk.write(0);
    si.write(1);
    clk.write(1);
    si.write(0);

    for (uint8_t i = 0; i < TSL1401CL_PIXEL_COUNT; i++) {
        clk.write(0);
        data[i] = adc.read_u16();
        clk.write(1);
    }
    clk.write(0);
}

void TSL1401CL::setIntegrationTime(uint32_t int_time_desired)
{
    integration_time = int_time_desired;
}

int TSL1401CL::findLineEdge (uint16_t threshold, uint8_t precision, size_t crop_amount, bool invert)
{
    if(precision == 0) precision = 1;

    // Create an array with resolution determined by precision containing the change between points in data
    uint16_t change_len = (TSL1401CL_PIXEL_COUNT - 2*crop_amount)/precision - 1;
    int32_t change_in_data[change_len];
    for(uint16_t i = crop_amount, j = 0; i < TSL1401CL_PIXEL_COUNT - crop_amount - precision; i += precision, j++) {
        change_in_data[j] = (int)data[i+precision] - (int)data[i];
    }

    // Find the index of the maximum value in change_in_data
    uint16_t maxchange = 0;
    for(uint16_t i = 1; i < change_len; i++) {
        if(invert) {
            if(change_in_data[i] < change_in_data[maxchange]) maxchange = i;
        } else {
            if(change_in_data[i] > change_in_data[maxchange]) maxchange = i;
        }
    }

    // If there isn't a sharp enough change returns -1
    if((change_in_data[maxchange] >= threshold && !invert) || (change_in_data[maxchange]*-1 >= threshold && invert)) {
        return maxchange*precision + crop_amount;
    }
    else return -1;
}

int TSL1401CL::findLineCenter (uint16_t threshold, uint8_t precision, size_t crop_amount)
{
    if(precision == 0) precision = 1;

    // Create an array with resolution determined by precision containing the change between points in data
    uint16_t change_len = (TSL1401CL_PIXEL_COUNT - 2*crop_amount)/precision - 1;
    int32_t change_in_data[change_len];
    for(uint16_t i = crop_amount, j = 0; i < TSL1401CL_PIXEL_COUNT - crop_amount - precision; i += precision, j++) {
        change_in_data[j] = (int)data[i+precision] - (int)data[i];
    }

    // Find the index of the maximum and minimum value in change_in_data
    uint16_t maxchange = 0;
    uint16_t minchange = 0;
    for(uint16_t i = 1; i < change_len; i++) {
        if(change_in_data[i] > change_in_data[maxchange]) maxchange = i;
        if(change_in_data[i] < change_in_data[minchange]) minchange = i;
    }

    // Return the average of the max and min value
    if(change_in_data[maxchange] >= threshold && change_in_data[minchange]*-1 >= threshold) {
        return (precision*maxchange+precision*minchange)/2 + crop_amount;
    }
    return -1;
}

uint32_t TSL1401CL::getData(uint8_t index)
{
    return data[index];
}