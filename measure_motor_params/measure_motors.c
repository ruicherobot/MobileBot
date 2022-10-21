/*******************************************************************************
* measure_motor_params.c
*   Template code 
*   Complete this code to automatically measure motor parameters
*   or print out data to be namalyzed in numpy
* 
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/encoder.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/motor.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"

float enc2meters = (WHEEL_DIAMETER * M_PI) / ENCODER_RES;

void test_speed(float du, float dtime_s);

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(int argc, char **argv)
{
    if (argc > 3)
    {
        fprintf(stderr, "ERROR\n");
        return 1;
    }
    int lr = atoi(argv[1]);
    int direction = atoi(argv[2]);

    // make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if (rc_kill_existing_process(2.0) < -2)
        return -1;

    // start signal handler so we can exit cleanly
    if (rc_enable_signal_handler() == -1)
    {
        fprintf(stderr, "ERROR: failed to start signal handler\n");
        return -1;
    }

#if defined(MRC_VERSION_1v3) || defined(MRC_VERSION_2v1)
    if (mb_motor_init() < 0)
    {
        fprintf(stderr, "ERROR: failed to initialze mb_motors\n");
        return -1;
    }
#endif

#if defined(BEAGLEBONE_BLUE)
    if (rc_motor_init() < 0)
    {
        fprintf(stderr, "ERROR: failed to initialze motors\n");
        return -1;
    }
#endif

    if (rc_encoder_eqep_init() < 0)
    {
        fprintf(stderr, "ERROR: failed to initialze encoders\n");
        return -1;
    }

    // make PID file to indicate your project is running
    // due to the check made on the call to rc_kill_existing_process() above
    // we can be fairly confident there is no PID file already and we can
    // make our own safely.
    rc_make_pid_file();

    // done initializing so set state to RUNNING
    rc_set_state(RUNNING);

    if (rc_get_state() == RUNNING)
    {
        rc_encoder_eqep_init();
        int enc_reading = 0;
        rc_motor_init();
        rc_nanosleep(1E9); //sleep for 1s
        FILE *fp = NULL;
        fp = fopen("/home/debian/mobilebot/measure_motor_params/calibration_data.csv", "w");
        double i = 0;

        lr = 1;
        direction = 1;

        for (i = 0.00; i <= 1; i = i + 0.01)
        {
            rc_encoder_eqep_write(lr, 0);
            rc_motor_set(lr, direction * i);

            rc_nanosleep(1E9);
            enc_reading = rc_encoder_eqep_read(lr);
            //rc_encoder_eqep_write(lr, 0);
            rc_motor_set(lr, 0.0);
            printf("%f \n", enc_reading * enc2meters);
            //fprintf(fp, "%lf\t%lf\n", i, enc_reading * enc2meters);
        }

        rc_motor_set(1, 0.0);
        rc_motor_set(2, 0.0);
        rc_nanosleep(1E9);

        lr = 1;
        direction = -1;

        for (i = 0.00; i <= 1; i = i + 0.01)
        {
            rc_encoder_eqep_write(lr, 0);
            rc_motor_set(lr, direction * i);

            rc_nanosleep(1E9);
            enc_reading = rc_encoder_eqep_read(lr);
            //rc_encoder_eqep_write(lr, 0);
            rc_motor_set(lr, 0.0);
            printf("%f \n", enc_reading * enc2meters);
            //fprintf(fp, "%lf\t%lf\n", i, enc_reading * enc2meters);
        }

        rc_motor_set(1, 0.0);
        rc_motor_set(2, 0.0);
        rc_nanosleep(1E9);

        lr = 2;
        direction = 1;

        for (i = 0.00; i <= 1; i = i + 0.01)
        {
            rc_encoder_eqep_write(lr, 0);
            rc_motor_set(lr, direction * i);

            rc_nanosleep(1E9);
            enc_reading = rc_encoder_eqep_read(lr);
            //rc_encoder_eqep_write(lr, 0);
            rc_motor_set(lr, 0.0);
            printf("%f \n", enc_reading * enc2meters);
            //fprintf(fp, "%lf\t%lf\n", i, enc_reading * enc2meters);
        }

        rc_motor_set(1, 0.0);
        rc_motor_set(2, 0.0);
        rc_nanosleep(1E9);

        lr = 2;
        direction = -1;

        for (i = 0.00; i <= 1; i = i + 0.01)
        {
            rc_encoder_eqep_write(lr, 0);
            rc_motor_set(lr, direction * i);

            rc_nanosleep(1E9);
            enc_reading = rc_encoder_eqep_read(lr);
            //rc_encoder_eqep_write(lr, 0);
            rc_motor_set(lr, 0.0);
            printf("%f \n", enc_reading * enc2meters);
            //fprintf(fp, "%lf\t%lf\n", i, enc_reading * enc2meters);
        }

        rc_motor_set(1, 0.0);
        rc_motor_set(2, 0.0);
        rc_nanosleep(1E9);

        fclose(fp);
    }

    // TODO: Plase exit routine here

    rc_encoder_eqep_cleanup();
    rc_motor_cleanup();

    // remove pid file LAST
    rc_remove_pid_file();
    return 0;
}

void test_speed(float duty, float dtime_s)
{
}