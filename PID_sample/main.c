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

int main(int argc, const char * argv[]) {
    // insert code here...
    
    Kp = atof(argv[1]);
    Ki = atof(argv[2]);
    Kd = atof(argv[3]);
    
    printf("Kp = %f, Ki = %f, Kd = %f\n", Kp, Ki, Kd);

    float position = 0.21;
    float control = 0;
    int adjustCnt = 0;
    while (fabs(position - setPoint) > 0.1) {
        control = PIDcal(setPoint, position);
        position = position + control;
        adjustCnt++;
        printf("Adjust# = %d, control = %f, position = %f\n", adjustCnt, control, position);
    }
    return 0;
}

//100ms loop time //ForCurrentSaturation

#define epsilon 0.01
#define dt 0.01
#define MAX 4
#define MIN -4
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

#define mcu_pid_epsilon 0.01
#define mcu_pid_dt 1/25000*50 // PWM rate 25 kHz, check period 50
#define mcu_pid_MAX (65355 * 0.05)
#define mcu_pid_MIN -mcu_pid_MAX

float mcuActuatorPIDdac(float setpoint, float actual_step)
{
    static float pre_error = 0;
    static float integral = 0;
    float error;
    float derivative;
    float output;
    
    //Caculate P,I,D
    error = setpoint - actual_step;
    
    //In case of error too small then stop integration
    if(fabsf(error) > mcu_pid_epsilon)
    {
        integral = integral + error*dt;
    }
    
    derivative = (error - pre_error)/dt;
    output = Kp*error + Ki*integral + Kd*derivative;
    
    //Saturation Filter
    if(output > mcu_pid_MAX)
    {
        output = mcu_pid_MAX;
    }
    else if(output < mcu_pid_MIN)
    {
        output = MIN;
    }
    //Update error pre_error = error;
    return output;
}
