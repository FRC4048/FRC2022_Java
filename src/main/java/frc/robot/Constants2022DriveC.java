// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public interface Constants2022DriveC extends GameConstants {

  //Global Constant
  public static final boolean ENABLE_DEBUG = true;

  //CAN ID
  public static final int PDP_CAN_ID = 0;
  public static final int DRIVE_LEFT1_ID = 8;
  public static final int DRIVE_LEFT2_ID = 7;
  public static final int DRIVE_RIGHT1_ID = 3;
  public static final int DRIVE_RIGHT2_ID = 6;
  public static final int PCM_CAN_ID = 20;
  public static final int SHOOTER_MOTOR_ID = 11;
  public static final int INTAKE_MOTOR_ID = 10;
  public static final int TURRET_MOTOR_ID = 1;
  public static final int PIGEON_CAN_ID = 7;
  public static final int HOOD_MOTOR_ID = 8;
  public static final int CLIMBER_RIGHT_WINCH_ID = 1;
  public static final int CLIMBER_LEFT_WINCH_ID = 2;
  public static final int CLIMBER_RIGHT_ARM_ID = 5;
  public static final int CLIMBER_LEFT_ARM_ID = 10;

  //PDP
  public static final int PDP_DRIVE_L1 = 13;
  public static final int PDP_DRIVE_L2 = 15;
  public static final int PDP_DRIVE_R1 = 0;
  public static final int PDP_DRIVE_R2 = 1;
  public static final int PDP_CLIMBER_L_WINCH = 50;
  public static final int PDP_CLIMBER_R_WINCH = 51;
  public static final int PDP_CLIMBER_L_ARM = 52;
  public static final int PDP_CLIMBER_R_ARM = 53;

  //DIO
  public static final int INTAKE_SENSOR_ID_1 = 0; //temp
  public static final int INTAKE_SENSOR_ID_2 = 1;
  public static final int CLIMBER_L_WINCH_SENSOR = 4;
  public static final int CLIMBER_R_WINCH_SENSOR = 5;

  //Relay

  //PCM
  public static final int SHOOTER_PISTON_ID = 3;
  public static final int INTAKE_SOLENOID_1 = 0;
  public static final int INTAKE_SOLENOID_2 = 4;
  public static final int CLIMBER_L_PISTON_ID = 6;
  public static final int CLIMBER_R_PISTON_ID = 7;

  // Elevator
  public static final int ELEVATOR_PISON_ID = 3;

  // Limelight
  public static final String LIMELIGHT_IP_ADDR = "10.40.48.34";

}

