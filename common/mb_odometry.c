/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry and dead rekoning 
*
*******************************************************************************/

#include "../mobilebot/mobilebot.h"
#include "mb_defs.h"
#include <math.h>

#define PI 3.14159265358979323846

/*******************************************************************************
* mb_initialize_odometry() 
*
* TODO: initialize odometry
* NOTE: you should initialize from Optitrack data if available
*
*******************************************************************************/
void mb_initialize_odometry(mb_odometry_t *mb_odometry, float x, float y, float theta)
{
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->theta = theta;
}

/*******************************************************************************
* mb_update_odometry() 
*
* TODO: calculate odometry from internal variables
*       publish new odometry to lcm ODOMETRY_CHANNEL
*
*******************************************************************************/
void mb_update_odometry(mb_odometry_t *mb_odometry, mb_state_t *mb_state)
{
    float enc2meters = (WHEEL_DIAMETER * PI) / ENCODER_RES;
    float theta_delta_odo = (mb_state->right_encoder_delta * RIGHT_ENC_POL - mb_state->left_encoder_delta * LEFT_ENC_POL) * enc2meters / WHEEL_BASE;
    float theta_delta_gyro = mb_state->tb_angles[2] - mb_state->last_yaw;
    float theta_delta = theta_delta_odo;
    float theta_thereshold = 0.01;
    if ((theta_delta_odo - theta_delta_gyro < -theta_thereshold) && (theta_delta_odo - theta_delta_gyro > theta_thereshold)) {
        theta_delta = theta_delta_gyro;
        printf("USE theta_delta_GYRO now!\n");
    }
    float d_delta = (mb_state->right_encoder_delta * RIGHT_ENC_POL + mb_state->left_encoder_delta * LEFT_ENC_POL) * enc2meters / 2.0;
    mb_odometry->x += d_delta * cos(mb_odometry->theta + theta_delta / 2);
    mb_odometry->y += d_delta * sin(mb_odometry->theta + theta_delta / 2);
    mb_odometry->theta += theta_delta;
    mb_odometry->theta = mb_clamp_radians(mb_odometry->theta);
}

/*******************************************************************************
* mb_clamp_radians() 
* clamp an angle from -PI to PI
*******************************************************************************/
float mb_clamp_radians(float angle)
{

    if (angle < -PI)
    {
        for (; angle < -PI; angle += 2.0 * PI)
            ;
    }
    else if (angle > PI)
    {
        for (; angle > PI; angle -= 2.0 * PI)
            ;
    }

    return angle;
}

/*******************************************************************************
* mb_angle_diff_radians() 
* computes difference between 2 angles and wraps from -PI to PI
*******************************************************************************/
float mb_angle_diff_radians(float angle1, float angle2)
{
    float diff = angle2 - angle1;
    while (diff < -PI)
        diff += 2.0 * PI;
    while (diff > PI)
        diff -= 2.0 * PI;
    return diff;
}