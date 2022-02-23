// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public interface Constants2022Test {
    //Global Constant
    public static final boolean ENABLE_DEBUG = true;

    //CAN ID
    public static final int PDP_CAN_ID = 0;

    public static final int INTAKE_MOTOR_ID = 10;
    public static final int DRIVE_LEFT1_ID = 3;
    public static final int DRIVE_LEFT2_ID = 4;
    public static final int DRIVE_RIGHT1_ID = 5;
    public static final int DRIVE_RIGHT2_ID = 2;
    public static final int PCM_CAN_ID = 20;
    public static final int SHOOTER_MOTOR_ID = 5; //Placeholder
    public static final int TURRET_MOTOR_ID = 10;

    //PDP
    public static final int PDP_DRIVE_L1 = 13; //Placeholder
    public static final int PDP_DRIVE_L2 = 15; //Placeholder
    public static final int PDP_DRIVE_R1 = 0; //Placeholder
    public static final int PDP_DRIVE_R2 = 1; //Placeholder

    //DIO
    public static final int INTAKE_SENSOR_ID = 0;

    //Relay

    //PCM
    public static final int SHOOTER_PISTON_ID = 3; //Placeholder
    public static final int INTAKE_SOLENOID_1 = 0;
    public static final int INTAKE_SOLENOID_2 = 3;
    
    //DRIVETRAIN CONSTANTS

    //Shooter
    public static final double SHOOTER_CLOCKWISE_SPEED = 0.5;
    public static final double SHOOTER_COUNTERCLOCKWISE_SPEED = -0.5;

    //Turret CONSTANTS
    public static final double TURRETSPIN_SCALEFACTOR = 0.5;

    //Hood
    public static final double HOOD_MOTOR_SPEED = 0.75;
    public static final int HOOD_MOTOR_ID = 10;
    public static final double HOOD_AUTO_MOTOR_SPEED = 0.5;
    public static final double HOOD_AUTO_LIMIT = 2;
    public static final double HOOD_JOYSTICK_THRESHOLD = 0.2;

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
  
    //Limelight Settings
    public static final double CAMERA_HEIGHT = 20.9;
    public static final double TARGET_HEIGHT = 19.8;
    //cameraAngle is a placeholder value
    public static final double CAMERA_ANGLE = 18.7;
}
