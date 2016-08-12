/*
 * A sample path following program for the Zumy board.
 * Author: Galen Savidge
 */

#include "mbed.h"
#include "TSL1401CL.h"

/***** Constants ******/
#define CAM_INTEGRATION_TIME 80

#define LINE_THRESHOLD 12000
#define LINE_PRECISION 2
#define LINE_CROP_AMOUNT 4

#define SPEED_PWM 0.125
#define TURN_SENS_INNER 1.5F
#define TURN_SENS_OUTER 0.5F

#define SENSOR_FOV_MM 20
#define ZUMY_LENGTH_MM 40

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

/***** Helper functions *****/
float max(float a, float b);

// Sets the 4 LEDS on the mbed board to the four given binary values
void setLEDs(uint8_t a, uint8_t b, uint8_t c, uint8_t d);

// Steers by linearly decreasing the inner wheel speed as the line moves from the center
void steerStupid(int8_t line_pos);

// Based on steerStupid; adds the ability for the inner wheel to rotate in reverse
void steerPointTurns(int8_t line_pos);

// Based on steerPointTurns; the outer wheel moves faster the farther the line is from center
void steerImprovedPointTurns(int8_t line_pos);

int main()
{
    /********** SENSOR SETUP **********/
    TSL1401CL cam(clk, si, adc);
    cam.setIntegrationTime(CAM_INTEGRATION_TIME);
    /*
    int line_hist[16];
    uint16_t hist_index = 0;

    Timer loop_timer;
    loop_timer.start();
    uint32_t last_loop_time;
    */

    int8_t line_pos = -1;
    int8_t line_pos_previous = -1;

    // Read garbage data
    while(!cam.integrationReady());
    cam.read();

    while(1) {
        /***** Read the line sensor *****/
        while(!cam.integrationReady()); // Wait for integration
        cam.read();

        /***** Camera testing *****/
        /*if(hist_index >= 1000) {
            line_hist[hist_index - 1000] = cam.findLineEdge(LINE_THRESHOLD, LINE_PRECISION, LINE_CROP_AMOUNT);
        }
        hist_index++;
        if(hist_index == 1016) {
            pc.printf("---------- Raw camera data ----------\n");
            for(int i = 0; i < TSL1401CL_PIXEL_COUNT; i++) {
                pc.printf("%d\n",cam.getData(i));
            }

            pc.printf("---------- Line positions ----------\n");
            for(int i = 1000; i < 1016; i++) {
                pc.printf("%d\n",line_hist[i - 1000]);
            }
            pc.printf("Loop time: %d\n", last_loop_time);

            hist_index = 0;
        }*/

        /***** Line following loop *****/

        // Test to see if both edges are in the sensor FOV, and if they aren't, check
        line_pos = cam.findLineEdge(LINE_THRESHOLD, LINE_PRECISION, LINE_CROP_AMOUNT);
        
        //line_pos = cam.findLineCenter(LINE_THRESHOLD, LINE_PRECISION, LINE_CROP_AMOUNT);
        //if(line_pos == -1) line_pos = cam.findLineEdge(LINE_THRESHOLD, LINE_PRECISION, LINE_CROP_AMOUNT, false);
        //if(line_pos == -1) line_pos = cam.findLineEdge(LINE_THRESHOLD, LINE_PRECISION, LINE_CROP_AMOUNT, true);


        if(line_pos != -1) {
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

            // Steer the vehicle
            steerImprovedPointTurns(line_pos);

            line_pos_previous = line_pos;
        } else {
            // Lost the line
            setLEDs(1, 1, 1, 1);
            
            // Steer at maximum turn angle towards the last known line direction
            if(line_pos_previous >= TSL1401CL_PIXEL_COUNT/2) {
                steerImprovedPointTurns(TSL1401CL_PIXEL_COUNT - 1 - LINE_CROP_AMOUNT);
            } else {
                steerImprovedPointTurns(LINE_CROP_AMOUNT);
            }
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

    float pwm_outer = SPEED_PWM*(1.0F + TURN_SENS_OUTER*abs(line_pos)*2.0F/(float)TSL1401CL_PIXEL_COUNT);
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