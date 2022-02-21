// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public interface Constants2022Test {
    //Global Constant
    public static final boolean ENABLE_DEBUG = true;

    //CAN ID
    public static final int PDP_CAN_ID = 0;
    public static final int DRIVE_LEFT1_ID = 3;
    public static final int DRIVE_LEFT2_ID = 4;
    public static final int DRIVE_RIGHT1_ID = 5;
    public static final int DRIVE_RIGHT2_ID = 2;
    public static final int PCM_CAN_ID = 20;
    public static final int SHOOTER_MOTOR_ID = 11;
    public static final int TURRET_MOTOR_ID = 10;

    //PDP
    public static final int PDP_DRIVE_L1 = 13;
    public static final int PDP_DRIVE_L2 = 15;
    public static final int PDP_DRIVE_R1 = 0;
    public static final int PDP_DRIVE_R2 = 1;

    //DIO

    //Relay

    //PCM
    public static final int SHOOTER_PISTON_ID = 3;
    
    //DRIVETRAIN CONSTANTS

    //Shooter
    public static final double SHOOTER_CLOCKWISE_SPEED = 0.5;
    public static final double SHOOTER_COUNTERCLOCKWISE_SPEED = -0.5;

    //Turret CONSTANTS
    public static final double TURRETSPIN_SCALEFACTOR = 0.5;

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
    

    //Limelight Settings
    public static final double CAMERA_HEIGHT = 20.9;
    public static final double TARGET_HEIGHT = 19.8;
    //cameraAngle is a placeholder value
    public static final double CAMERA_ANGLE = 18.7;
}
