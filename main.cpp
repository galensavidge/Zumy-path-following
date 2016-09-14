/*
 * A sample path following program for the Zumy board.
 * Author: Galen Savidge
 *
 * Behavior: Zumy moves along the track until no line is found for LINE_END_TIME
 * cycles. Line position is recorded every frame as an 8 bit unsigned int. Line
 * position data is then output to serial.
 */

#include "mbed.h"
#include "TSL1401CL.h"

/***** Constants ******/
#define CAM_INTEGRATION_TIME 80

#define LINE_THRESHOLD 3000
#define LINE_PRECISION 2
#define LINE_CROP_AMOUNT 4

#define SPEED_PWM 0.125
#define TURN_SENS_INNER 1.5F
#define TURN_SENS_OUTER 0.5F

#define LINE_HIST_SIZE 12000
#define LINE_END_TIME 25

// Sensor pins
#define clk p16
#define si p17
#define adc p18

// Motors
PwmOut m1_fwd(p21);
PwmOut m1_back(p22);

PwmOut m2_fwd(p23);
PwmOut m2_back(p24);

// LEDs
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

// USB serial
Serial pc(USBTX, USBRX);

// Data storage
uint8_t line_hist[LINE_HIST_SIZE];
uint16_t hist_index;

/***** Helper functions *****/
float max(float a, float b);

// Sets the 4 LEDS on the mbed board to the four given binary values
void setLEDs(uint8_t a, uint8_t b, uint8_t c, uint8_t d);

// Steers by linearly decreasing the inner wheel speed as the line moves from the center (unused in this program)
void steerStupid(int8_t line_pos);

// Based on steerStupid; adds the ability for the inner wheel to rotate in reverse (unused in this program)
void steerPointTurns(int8_t line_pos);

// Based on steerPointTurns; the outer wheel moves faster the farther the line is from center
void steerImprovedPointTurns(int8_t line_pos);

int main()
{
    /********** SENSOR SETUP **********/
    TSL1401CL cam(clk, si, adc);
    cam.setIntegrationTime(CAM_INTEGRATION_TIME);
    
    int8_t line_pos = -1;
    int8_t line_pos_previous = -1;
    uint8_t line_lost_time = 0;
    
    hist_index = 0;

    // Read garbage data
    while(!cam.integrationReady());
    cam.read();

    while(1) {
        /***** Read line sensor *****/
        while(!cam.integrationReady()); // Wait for integration
        cam.read();

        /***** Line following loop *****/

        line_pos = cam.findLineEdge(LINE_THRESHOLD, LINE_PRECISION, LINE_CROP_AMOUNT);

        if(line_pos != -1) { // On the line
            // Set LEDs based on the position of the line
            if(line_pos < (TSL1401CL_PIXEL_COUNT - 2*LINE_CROP_AMOUNT)/4) {
                setLEDs(1, 0, 0, 0);
            } else if(line_pos < (TSL1401CL_PIXEL_COUNT - 2*LINE_CROP_AMOUNT)/2) {
                setLEDs(0, 1, 0, 0);
            } else if(line_pos < (TSL1401CL_PIXEL_COUNT - 2*LINE_CROP_AMOUNT)*3/4) {
                setLEDs(0, 0, 1, 0);
            } else {
                setLEDs(0, 0, 0, 1);
            }
            
            // Record the line position
            line_hist[hist_index] = line_pos;
            hist_index++;
            line_lost_time = 0;
            
            // Steer the vehicle
            steerImprovedPointTurns(line_pos);

            line_pos_previous = line_pos;
        }
        else { // Lost the line
            setLEDs(1, 1, 1, 1);
            if(abs(line_pos_previous - TSL1401CL_PIXEL_COUNT/2) > TSL1401CL_PIXEL_COUNT/4) {
                // Steer at maximum turn angle towards the last known line direction
                if(line_pos_previous >= TSL1401CL_PIXEL_COUNT/2) {
                    steerImprovedPointTurns(TSL1401CL_PIXEL_COUNT - 1 - LINE_CROP_AMOUNT);
                    //line_hist[hist_index] = TSL1401CL_PIXEL_COUNT - 1 - LINE_CROP_AMOUNT;
                }
                else {
                    steerImprovedPointTurns(LINE_CROP_AMOUNT);
                    //line_hist[hist_index] = LINE_CROP_AMOUNT;
                }
                line_hist[hist_index] = 0;
                hist_index++;
            }
            else {
                line_lost_time++;
                if(line_lost_time >= LINE_END_TIME) {
                    // Line end; transmit data to PC
                    m1_fwd.write(0);
                    m2_fwd.write(0);
                    while(!pc.readable());
                    pc.printf("----- Start line data -----\n");
                    for(int i = 0; i <= hist_index; i++) {
                        pc.printf("%d\n",line_hist[i]);
                    }
                    while(1);
                }
            }
        }
        if(hist_index > LINE_HIST_SIZE) {
            hist_index = 0;
        }
    }
}

float max(float a, float b)
{
    if(a >= b) return a;
    return b;
}

void setLEDs(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    led1 = a;
    led2 = b;
    led3 = c;
    led4 = d;
}

void steerStupid(int8_t line_pos)
{
    line_pos -= TSL1401CL_PIXEL_COUNT/2; // Find offset from center

    if(line_pos < 0) {
        m1_fwd.write(SPEED_PWM);
        m1_back.write(0);
        m2_fwd.write(max(SPEED_PWM*(1.0F + TURN_SENS_INNER*line_pos*2.0F/(float)TSL1401CL_PIXEL_COUNT), 0));
        m2_back.write(0);
    }

    if(line_pos >= 0) {
        m2_fwd.write(SPEED_PWM);
        m2_back.write(0);
        m1_fwd.write(max(SPEED_PWM*(1.0F - TURN_SENS_INNER*line_pos*2.0F/(float)TSL1401CL_PIXEL_COUNT), 0));
        m1_back.write(0);
    }
}

void steerPointTurns(int8_t line_pos)
{
    line_pos -= TSL1401CL_PIXEL_COUNT/2; // Find offset from center

    float pwm_outer = SPEED_PWM;
    float pwm_inner = SPEED_PWM*(1.0F - TURN_SENS_INNER*abs(line_pos)*2.0F/(float)TSL1401CL_PIXEL_COUNT);

    if(line_pos < 0) {
        m1_fwd.write(pwm_outer);
        m1_back.write(0);
        if(pwm_inner >= 0) {
            m2_fwd.write(pwm_inner);
            m2_back.write(0);
        } else {
            m2_fwd.write(0);
            m2_back.write(pwm_inner*-1.0F);
        }
    }

    if(line_pos >= 0) {
        m2_fwd.write(pwm_outer);
        m2_back.write(0);
        if(pwm_inner >= 0) {
            m1_fwd.write(pwm_inner);
            m1_back.write(0);
        } else {
            m1_fwd.write(0);
            m1_back.write(pwm_inner*-1.0F);
        }
    }
}

void steerImprovedPointTurns(int8_t line_pos)
{
    line_pos -= TSL1401CL_PIXEL_COUNT/2; // Find offset from center
    
    // Find desired motor voltages based on the controller
    float pwm_outer = SPEED_PWM*(1.0F + TURN_SENS_OUTER*abs(line_pos)*2.0F/(float)TSL1401CL_PIXEL_COUNT);
    float pwm_inner = SPEED_PWM*(1.0F - TURN_SENS_INNER*abs(line_pos)*2.0F/(float)TSL1401CL_PIXEL_COUNT);

    // Write to the appropriate PWM pins
    if(line_pos < 0) {
        m1_fwd.write(pwm_outer);
        m1_back.write(0);
        if(pwm_inner >= 0) {
            m2_fwd.write(pwm_inner);
            m2_back.write(0);
        } else {
            m2_fwd.write(0);
            m2_back.write(pwm_inner*-1.0F);
        }
    }

    if(line_pos >= 0) {
        m2_fwd.write(pwm_outer);
        m2_back.write(0);
        if(pwm_inner >= 0) {
            m1_fwd.write(pwm_inner);
            m1_back.write(0);
        } else {
            m1_fwd.write(0);
            m1_back.write(pwm_inner*-1.0F);
        }
    }
}