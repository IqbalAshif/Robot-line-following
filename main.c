#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>
#include <stdlib.h>
//function for left turn after robot goes off line to right side
void tank_turn_left(uint8 left_speed, uint8 right_speed, uint32 delay)
{
    MotorDirLeft_Write(1);
    MotorDirRight_Write(0);
    PWM_WriteCompare1(left_speed);
    PWM_WriteCompare2(right_speed);
    vTaskDelay(delay);
}
//function for right turn after robot goes off to left side
void tank_turn_right(uint8 left_speed, uint8 right_speed, uint32 delay)
{
    MotorDirLeft_Write(0);
    MotorDirRight_Write(1);
    PWM_WriteCompare1(left_speed);
    PWM_WriteCompare2(right_speed);
    vTaskDelay(delay);
}
void robot_run(bool run) //function for running the robot through the line
{
    reflectance_start();
    reflectance_set_threshold(13000, 13000, 13000, 13000, 13000, 13000);
    struct sensors_ digi;
    reflectance_digital(&digi);

    motor_forward(200, 1);

    if (run == false)
    {
        while (digi.l3 == 0 || digi.r3 == 0)
        {
            motor_forward(200, 1);
            reflectance_digital(&digi);
        }
    }
    else if (run == true)
    {
        while (digi.l3 == 1 || digi.r3 == 1)
        {
            motor_forward(200, 1);
            reflectance_digital(&digi);
        }
    }
}

#if 1
void zmain(void)
{
    TickType_t start = 0;
    TickType_t end = 0;
    TickType_t off_the_line = 0;
    TickType_t back_on_line = 0;
    bool back_on_track = false; //to bring the robot back to the line
    bool on_track = true;       //robot is on line or not
    printf ("\nBoot\n");
    send_mqtt ("Zumo03/boot", "zumbo");	//At boot
    IR_Start();
    struct sensors_ dig;
    vTaskDelay(100);
    while (SW1_Read())
        vTaskDelay(100);
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000); // set center sensor threshold to 11000 and others to 9000
    motor_start();
    bool run = false;
    ;
    robot_run(run);
    motor_forward(0, 0);
    IR_flush();
    send_mqtt("Zumo03/ready", "line");
    start = xTaskGetTickCount();             //time at giving IR command
    print_mqtt("Zumo03/start", "%d", start); //printing start time when pressing the IR remote button
    IR_wait();
    run = true;
    robot_run(run);
    motor_forward(255, 10);
    while (true)
    {
        reflectance_digital(&dig);
        if (dig.l3 == 1 && dig.r3 == 1 && dig.l2 == 1 && dig.r2 == 1 && dig.l1 == 1 && dig.r1 == 1) //on black
        {
            run = true;
            robot_run(run); //not stopping at the first black line
            run = false;
            robot_run(run);                               //stopping at the second black line
            end = xTaskGetTickCount();                    //to get the end time
            print_mqtt("Zumo03/stop", "%d", end);         //printing the end time
            print_mqtt("Zumo03/time", "%d", end - start); //execution time

            while (true)
            {
                motor_forward(0, 0);
            }
        }
        else if (dig.l1 == 1 && dig.r1 == 1) //sensors in the middle are black
        {
            if (on_track == false)
            {
                motor_forward(255, 1);
                back_on_line = xTaskGetTickCount();
                print_mqtt("Zumo03/line", "%d", back_on_line); //printime time when robot is back on line
                on_track = true;
            }
            else
            {
                motor_forward(255, 1);
            }
        }
        else if ((dig.l2 == 1 || dig.l3 == 1) && dig.r3 == 0) //robot turning right
        {

            if (dig.l1 == 0 && dig.r1 == 0)
            {
                motor_turn(0, 255, 25);
                on_track = false;
                back_on_track = true;
                motor_turn(0, 255, 25);
                off_the_line = xTaskGetTickCount();
                print_mqtt("Zumo03/miss", "%d", off_the_line); //printing time when robot slides off the line
            }
            else
            {
                motor_turn(0, 255, 25);
                back_on_track = true;
            }
        }
        else if (dig.l3 == 0 && (dig.r2 == 1 || dig.r3 == 1)) //robot turning left
        {

            if (dig.l1 == 0 && dig.r1 == 0)
            {
                motor_turn(255, 0, 25);
                on_track = false;
                back_on_track = false;
                motor_turn(255, 0, 25);
                off_the_line = xTaskGetTickCount();
                print_mqtt("Zumo03/miss", "%d", off_the_line); //printing time when robot slides off the line
            }
            else
            {
                motor_turn(255, 0, 25);
                back_on_track = false;
            }
        }

        else if (dig.l1 == 0 && dig.r1 == 0 && dig.l2 == 0 && dig.r2 == 0 && dig.l3 == 0 && dig.r3 == 0) //off line
        {
            off_the_line = xTaskGetTickCount();
            print_mqtt("Zumo03/miss", "%d", off_the_line); //printing time when robot slides off the line
            on_track = false;

            if (back_on_track == true) //turning left if last turn was right
            {
                tank_turn_left(255, 255, 25);
            }
            else
            {
                tank_turn_right(255, 255, 25); //turning right if last turn was left
            }
        }
        else
        {
            motor_forward(255, 1);
        }
    }
    while (true)

    {
        vTaskDelay(100);
    }
}

#endif