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
    public static final boolean ENABLE_DEBUG = false;

    // THE CONSTANTS IN THIS INTERFACE ARE ALL COMMENTED OUT BECAUSE WE DON'T HAVE A ROBOT YET.
    // ONCE THE HARDWARE IS DELIVERED< ALL THESE NEED TO BE DEFINED AND TESTED

    //CAN ID
    public static final int PDP_CAN_ID = 0;
    public static final int DRIVE_LEFT1_ID = 40; 
    public static final int DRIVE_LEFT2_ID = 39;
    public static final int DRIVE_RIGHT1_ID = 38;
    public static final int DRIVE_RIGHT2_ID = 37;
    public static final int INTAKE_MOTOR_ID = 4;
    public static final int TURRET_MOTOR_ID = 9;
    public static final int PCM_CAN_ID = 20;
    public static final int SHOOTER_MOTOR_ID = 46;
    public static final int HOOD_MOTOR_ID = 10;
    public static final int CLIMBER_RIGHT_WINCH_ID = 34; 
    public static final int CLIMBER_LEFT_WINCH_ID = 43; 
    public static final int CLIMBER_RIGHT_ARM_ID = 12; 
    public static final int CLIMBER_LEFT_ARM_ID = 2; 


    //PDP
    public static final int PDP_DRIVE_L1 = 2; 
    public static final int PDP_DRIVE_L2 = 13; 
    public static final int PDP_DRIVE_R1 = 1; 
    public static final int PDP_DRIVE_R2 = 14; 
    public static final int PDP_CLIMBER_L_WINCH = 12;
    public static final int PDP_CLIMBER_R_WINCH = 15;
    public static final int PDP_CLIMBER_L_ARM = 8;
    public static final int PDP_CLIMBER_R_ARM = 9;

    //DIO
    public static final int INTAKE_SENSOR_ID_1 = 0; //temp
    public static final int INTAKE_SENSOR_ID_2 = 1;
    public static final int ELEVATOR_SENSOR_ID = 2;
    public static final int CLIMBER_L_WINCH_SENSOR = 9;
    public static final int CLIMBER_R_WINCH_SENSOR = 8;
    public static final int CLIMBER_L_HOOK_SENSOR = 4;
    public static final int CLIMBER_R_HOOK_SENSOR = 5;
    
    //AIO 
    public static final int HOOD_POTENTIOMETER = 0;

    //Relay

    //PCM
    public static final int SHOOTER_PISTON_ID = 0;
    public static final int INTAKE_SOLENOID_1 = 3;
    public static final int CLIMBER_L_PISTON_ID = 2;
    public static final int STOP_SOLENOID_ID = 1;   

    // Elevator
    
    //DRIVETRAIN CONSTANTS

    //Shooter

    public static final int PIGEON_CAN_ID = 7;

    // Limelight
    public static final String LIMELIGHT_IP_ADDR = "10.40.48.33";

    //Wheel
    public static final double WHEEL_RADIUS = 0.0762; // meters
    public static final double CHASSIS_GEAR_RATIO = 10.75; // this value should be x:1
}
