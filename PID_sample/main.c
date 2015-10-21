//
//  main.c
//  PID_sample
//
//  Created by Veli-Matti Kananen on 21.10.2015.
//  Copyright © 2015 Veli-Matti Kananen. All rights reserved.
//

#include <stdio.h>
#include <math.h> //fabsf
#include <stdlib.h> //atoi

float Kp = 0;
float Kd = 0;
float Ki = 0;
#define setPoint 100.0

static float PIDcal(float setpoint,float actual_position);
static float mcuActuatorPIDdac(float target, float actual);
static float takeStep(const float dac);


#define mcu_pid_epsilon 0.01
#define mcu_pid_dt 1/25000*50 // PWM rate 25 kHz, check period 50
#define mcu_pid_MAX (65355 * 0.05)
#define mcu_pid_MIN -mcu_pid_MAX
#define mcu_pid_setPoint 5.0
#define MCU_DAC_MAX 65535
#define MCU_DAC_MIN 512


int main(int argc, const char * argv[]) {
    // insert code here...
    
    Kp = atof(argv[1]);
    Ki = atof(argv[2]);
    Kd = atof(argv[3]);
    
    printf("Kp = %f, Ki = %f, Kd = %f\n", Kp, Ki, Kd);
/*
    float position = 0.21;
    float control = 0;
    int adjustCnt = 0;
    while (fabs(position - setPoint) > 0) {
        control = PIDcal(setPoint, position);
        position = position + control;
        adjustCnt++;
        printf("Adjust# = %d, control = %f, position = %f\n", adjustCnt, control, position);
    }
 */
    float current_dac = 65535.00/2.00;          // Initial value. Mid point of 16 bit dac.
    float actual_step = takeStep(current_dac);  // Actual journey taken with initial dac value.
    float delta_dac = 0;
    int adjustCnt = 0;
    
    while (fabs(actual_step - mcu_pid_setPoint) > 0) {
//        while (1) {
        delta_dac = mcuActuatorPIDdac(mcu_pid_setPoint, actual_step);
        current_dac = current_dac - delta_dac;

        if (current_dac > MCU_DAC_MAX)
            current_dac = MCU_DAC_MAX;
        else if(current_dac < MCU_DAC_MIN)
            current_dac = MCU_DAC_MIN;

        // Emulate stepping
        actual_step = takeStep(current_dac);
        
        // Take a step
        adjustCnt++;
        printf("Adjust# = %d, delta_dac = %f, actual_step = %f, current_dac = %f\n", adjustCnt, delta_dac, actual_step, current_dac);
    }

    
    return 0;
}

float mcuActuatorPIDdac(float target, float actual)
{
    static float pre_error = 0;
    static float integral = 0;
    float error;
    float derivative;
    float output;
    
    // Calculate P,I,D
    error = target - actual;
    
    // In case of error too small then stop integration
    if(fabsf(error) > mcu_pid_epsilon)
    {
        integral = integral + error*mcu_pid_dt;
    }
    
    derivative = (error - pre_error)/mcu_pid_dt;
    output = Kp*error + Ki*integral + Kd*derivative;
    
    //Saturation Filter
    if(output > mcu_pid_MAX)
    {
        output = mcu_pid_MAX;
    }
    else if(output < mcu_pid_MIN)
    {
        output = mcu_pid_MIN;
    }
    
    //Update error
    pre_error = error;
    
    return output;
}

float takeStep(const float dac)
{
    float retval;
    retval = (MCU_DAC_MAX-dac)/10; // Imagine that 10 DAC points == 1 nm
    return retval;
}



//100ms loop time //ForCurrentSaturation

#define epsilon 0.01
#define dt 0.01
#define MAX 10
#define MIN -10
//#define Kp 0.1
//#define Kd 0.01
//#define Ki 0.005


float PIDcal(float setpoint, float actual_position)
{
    static float pre_error = 0;
    static float integral = 0;
    float error;
    float derivative;
    float output;
    
    //Caculate P,I,D
    error = setpoint - actual_position;
    
    //In case of error too small then stop integration
    if(fabsf(error) > epsilon)
    {
        integral = integral + error*dt;
    }
    
    derivative = (error - pre_error)/dt;
    output = Kp*error + Ki*integral + Kd*derivative;
    
    //Saturation Filter
    if(output > MAX)
    {
        output = MAX;
    }
    else if(output < MIN)
    {
        output = MIN;
    }
    //Update error pre_error = error;
    return output;
}

