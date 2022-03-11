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
public interface Constants2022Robot extends GameConstants {
    //Global Constant
    public static final boolean ENABLE_DEBUG = true;

    // THE CONSTANTS IN THIS INTERFACE ARE ALL COMMENTED OUT BECAUSE WE DON'T HAVE A ROBOT YET.
    // ONCE THE HARDWARE IS DELIVERED< ALL THESE NEED TO BE DEFINED AND TESTED

    //CAN ID
    public static final int PDP_CAN_ID = 0;
    public static final int DRIVE_LEFT1_ID = 40;
    public static final int DRIVE_LEFT2_ID = 39;
    public static final int DRIVE_RIGHT1_ID = 38;
    public static final int DRIVE_RIGHT2_ID = 37;
    public static final int INTAKE_MOTOR_ID = 4;
    public static final int TURRET_MOTOR_ID = 1;
    public static final int PCM_CAN_ID = 20;
    public static final int TURRET_HOOD_CAN_ID = 12;
    public static final int SHOOTER_MOTOR_ID = 46;
    public static final int HOOD_MOTOR_ID = 8;
    public static final int CLIMBER_RIGHT_WINCH_ID = 1;
    public static final int CLIMBER_LEFT_WINCH_ID = 2;
    public static final int CLIMBER_RIGHT_MOTOR_ID = 5;
    public static final int CLIMBER_LEFT_MOTOR_ID = 10;


    //PDP
    public static final int PDP_DRIVE_L1 = 2; 
    public static final int PDP_DRIVE_L2 = 13; 
    public static final int PDP_DRIVE_R1 = 1; 
    public static final int PDP_DRIVE_R2 = 14; 

    //DIO
    public static final int INTAKE_SENSOR_ID_1 = 0; //temp
    public static final int INTAKE_SENSOR_ID_2 = 1;

    //Relay

    //PCM
    public static final int SHOOTER_PISTON_ID = 0;
    public static final int INTAKE_SOLENOID_1 = 3;
    public static final int INTAKE_SOLENOID_2 = 4;

    // Elevator
    
    //DRIVETRAIN CONSTANTS

    public static final int PIGEON_CAN_ID = 7;

    // LIMELIGHT
    public static final int LIMELIGHT_TARGET_DETECTION = 1;
    public static final int LIMELIGHT_STREAMING = 0;  
}
