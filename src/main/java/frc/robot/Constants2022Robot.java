// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public interface Constants2022Robot {
        //Global Constant
    public static final boolean ENABLE_DEBUG = false;

    //CAN ID
    public static final int PDP_CAN_ID = 0;
    public static final int MOTOR_LEFT1_ID = 1;
    public static final int MOTOR_LEFT2_ID = 2;
    public static final int MOTOR_RIGHT1_ID = 3;
    public static final int MOTOR_RIGHT2_ID = 4;

    public static final int PCM_CAN_ID = 20;
    public static final int SHOOTER_MOTOR_ID = 5; //Placeholder

    //PDP
    public static final int PDP_DRIVE_L1 = 13; //Placeholder
    public static final int PDP_DRIVE_L2 = 15; //Placeholder
    public static final int PDP_DRIVE_R1 = 0; //Placeholder
    public static final int PDP_DRIVE_R2 = 1; //Placeholder

    //DIO

    //Relay

    public static final int INTAKE_SENSOR_ID = 0;
    public static final int INTAKE_MOTOR_ID = 10;
    public static final int PCM_ID = 20;

    //PCM Channels
    public static final int INTAKE_SOLENOID_1 = 0;
    public static final int INTAKE_SOLENOID_2 = 3;


    //PCM
    public static final int SHOOTER_PISTON_ID = 3; //Placeholder
    
    //DRIVETRAIN CONSTANTS

    //SHOOTER CONSTANTS

    //OI
    public static final int XBOX_A_BUTTON = 1;
    public static final int XBOX_B_BUTTON = 2;
    public static final int XBOX_X_BUTTON = 3;
    public static final int XBOX_Y_BUTTON = 4;
    public static final int XBOX_LEFT_BUMPER = 5;
    public static final int XBOX_RIGHT_BUMPER = 6;
    public static final int XBOX_BACK_BUTTON = 7;
    public static final int XBOX_START_BUTTON = 8;
    public static final int XBOX_LEFT_STICK_PRESS = 9;
    public static final int XBOX_RIGHT_STICK_PRESS = 10;
    public static final int LEFT_JOYSTICK_ID = 0;
    public static final int RIGHT_JOYSTICK_ID = 1;

    public static final int CONTROLLER_ID = 2;


    //Motor Speeds
    public static final double INTAKE_MOTOR_SPEED = 1;

    //Timeouts
    public static final int DEPLOYED_INTAKE_TIMEOUT = 5;
    public static final int RAISED_INTAKE_TIMEOUT = 3;


}
