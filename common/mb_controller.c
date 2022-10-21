#include "../mobilebot/mobilebot.h"

/*******************************************************************************
* int mb_initialize()
*
* this initializes all the PID controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

int mb_initialize_controller()
{

    mb_load_controller_config();
    left_wheel_velocity_pid = rc_filter_empty();
    right_wheel_velocity_pid = rc_filter_empty();
    rc_filter_pid(&left_wheel_velocity_pid, left_pid_params.kp, left_pid_params.ki, left_pid_params.kd, left_pid_params.dFilterHz, DT);
    rc_filter_pid(&right_wheel_velocity_pid, right_pid_params.kp, right_pid_params.ki, right_pid_params.kd, right_pid_params.dFilterHz, DT);
    rc_filter_enable_saturation(&left_wheel_velocity_pid, -1.0, 1.0);
    rc_filter_enable_saturation(&right_wheel_velocity_pid, -1.0, 1.0);

    return 0;
}

/*******************************************************************************
* int mb_load_controller_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

int mb_load_controller_config()
{
    FILE *file = fopen("/home/debian/mobilebot/mobilebot/parameters_pid.cfg", "r");
    if (file == NULL)
    {
        printf("Error opening pid.cfg\n");
    }
    fscanf(file, "%f %f %f %f %f %f %f %f",
           &left_pid_params.kp,
           &left_pid_params.ki,
           &left_pid_params.kd,
           &left_pid_params.dFilterHz,
           &right_pid_params.kp,
           &right_pid_params.ki,
           &right_pid_params.kd,
           &right_pid_params.dFilterHz);

    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* TODO: Write your PID controller here
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t *mb_state, mb_setpoints_t *mb_setpoints)
{
    //  1 1    1 -1    2 1    2 -1
    float a[4] = {-1.0026, .9696, 0.961, -0.9827};
    float b[4] = {0.1119, -0.0977, -0.0738 , 0.0705};
    //PID Controler stuff
    float left_error = (mb_setpoints->fwd_velocity - WHEEL_BASE / 2 * mb_setpoints->turn_velocity) - mb_state->left_velocity;
    float right_error = (mb_setpoints->fwd_velocity + WHEEL_BASE / 2 * mb_setpoints->turn_velocity) - mb_state->right_velocity;
    float left_out = rc_filter_march(&left_wheel_velocity_pid, left_error);    //left velo
    float right_out = rc_filter_march(&right_wheel_velocity_pid, right_error); //right velo
    
    //PID Controler stuff
    //printf("LEFT_out  %f    RIGHT_out  %f\n", left_out, right_out);
    mb_state->left_cmd = mb_state->left_velocity + left_out;
    mb_state->right_cmd = mb_state->right_velocity + right_out;
    
    //Simple controler
    // mb_state->left_cmd = a[0] * mb_state->left_velocity + b[0];                         // calibration left
    // mb_state->right_cmd = a[4] * mb_state->right_velocity + b[4];                       // calibration right
    return 0;
}

/*******************************************************************************
* int mb_destroy_controller()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_destroy_controller()
{
    rc_filter_free(&left_wheel_velocity_pid);
    rc_filter_free(&right_wheel_velocity_pid);
    return 0;
}