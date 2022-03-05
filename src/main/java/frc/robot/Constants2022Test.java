// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public interface Constants2022Test extends GameConstants {
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
    public static final int HOOD_MOTOR_ID = 10;

    //PDP
    public static final int PDP_DRIVE_L1 = 13; //Placeholder
    public static final int PDP_DRIVE_L2 = 15; //Placeholder
    public static final int PDP_DRIVE_R1 = 0; //Placeholder
    public static final int PDP_DRIVE_R2 = 1; //Placeholder

    //DIO
    public static final int INTAKE_SENSOR_ID_1 = 0; //temp
    public static final int INTAKE_SENSOR_ID_2 = 1;

    //Relay

    //PCM
    public static final int SHOOTER_PISTON_ID = 3; //Placeholder
    public static final int INTAKE_SOLENOID_1 = 0;
    public static final int INTAKE_SOLENOID_2 = 3;  
    
    //Pigeon
    public static final int PIGEON_CAN_ID = 7;

    // LIMELIGHT
    public static final int LIMELIGHT_TARGET_DETECTION = 1;
    public static final int LIMELIGHT_STREAMING = 0;   
}
