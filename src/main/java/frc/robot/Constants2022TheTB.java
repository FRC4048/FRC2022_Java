// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public interface Constants2022TheTB extends GameConstants {

  //Global Constant
  public static final boolean ENABLE_DEBUG = true;

  //CAN ID
  public static final int PDP_CAN_ID = 0;

  public static final int DRIVE_LEFT1_ID = 44;
  public static final int DRIVE_LEFT2_ID = 45;
  public static final int DRIVE_RIGHT1_ID = 42;
  public static final int DRIVE_RIGHT2_ID = 41;

  public static final int PCM_CAN_ID = 20;
  public static final int SHOOTER_MOTOR_ID = 40;
  public static final int INTAKE_MOTOR_ID = 5;
  public static final int TURRET_MOTOR_ID = 12; //Don't know if this is right
  public static final int HOOD_MOTOR_ID = 10;

  //PDP
  public static final int PDP_DRIVE_L1 = 13; 
  public static final int PDP_DRIVE_L2 = 15; 
  public static final int PDP_DRIVE_R1 = 0; 
  public static final int PDP_DRIVE_R2 = 1; 

  //DIO
  public static final int INTAKE_SENSOR_ID_1 = 0; //temp
  public static final int INTAKE_SENSOR_ID_2 = 1; //temp

  //Relay

  //PCM
  public static final int SHOOTER_PISTON_ID = 7; //placeholder
  public static final int INTAKE_SOLENOID_1 = 0;
  public static final int INTAKE_SOLENOID_2 = 6;

  // Elevator
  public static final int ELEVATOR_PISON_ID = 3;

  //DRIVETRAIN CONSTANTS

  
  //Pigeon
  public static final int PIGEON_CAN_ID = 7;

  // LIMELIGHT
  public static final int LIMELIGHT_TARGET_DETECTION = 1;
  public static final int LIMELIGHT_STREAMING = 0;  

  //Autonomus *All Placehoders so far
  public static final double speed = 5;
  public static final double distance_Inches = 6;
  public static final int angle_Required = 20;

}
