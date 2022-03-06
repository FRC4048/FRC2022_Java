// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public interface Constants2022TheTB {

  //Global Constant
  public static final boolean ENABLE_DEBUG = true;

  //CAN ID
  public static final int PDP_CAN_ID = 0;

  public static final int DRIVE_LEFT1_ID = 44;
  public static final int DRIVE_LEFT2_ID = 45;
  public static final int DRIVE_RIGHT1_ID = 42;
  public static final int DRIVE_RIGHT2_ID = 41;
  public static final int PCM_CAN_ID = 20;
  public static final int SHOOTER_MOTOR_ID = 48;
  public static final int INTAKE_MOTOR_ID = 10;
  public static final int TURRET_MOTOR_ID = 12; //Don't know if this is right
  public static final int PIGEON_CAN_ID = 7;
  public static final int CLIMBER_RIGHT_WINCH_ID = 1;
  public static final int CLIMBER_LEFT_WINCH_ID = 2;
  public static final int CLIMBER_RIGHT_MOTOR_ID = 5;
  public static final int CLIMBER_LEFT_MOTOR_ID = 10;


  //PDP
  public static final int PDP_DRIVE_L1 = 13; 
  public static final int PDP_DRIVE_L2 = 15; 
  public static final int PDP_DRIVE_R1 = 0; 
  public static final int PDP_DRIVE_R2 = 1; 

  //DIO
  public static final int INTAKE_SENSOR_ID = 0;



  //Relay

  //PCM
  public static final int SHOOTER_PISTON_ID = 2; //placeholder
  public static final int INTAKE_SOLENOID_1 = 0;
  public static final int INTAKE_SOLENOID_2 = 1;
  public static final int CLIMBER_RIGHT_PISTON_ID = 6;
  public static final int CLIMBER_LEFT_PISTON_ID = 7;

  //DRIVETRAIN CONSTANTS

  //Shooter
  public static final double SHOOTER_CLOCKWISE_SPEED = 0.5;
  public static final double SHOOTER_COUNTERCLOCKWISE_SPEED = -0.5;
  
  // Intake
  public static final double INTAKE_BUFFER = 1.0;

  //Turret
  public static final double TURRETSPIN_SCALEFACTOR = 0.5;

  //Climber
  public static final double CLIMBER_ARM_SPEED = 1;
  public static final double CLIMBER_WINCH_SPEED = 1;
  public static final double CLIMBER_MIN_ARM_SPEED = 0.8;
  public static final double CLIMBER_MIN_WINCH_SPEED = 0.8;
  public static final double CLIMBER_TIMEOUT = 10;
  public static final double CLIMBER_MAX_ENCODER_DIFF = 1000;
  public static final double CLIMBER_ARM_TIMEOUT = 10;
  public static final double CLIMBER_WINCH_TIMEOUT = 10;


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
  public static final int CONTROLLER_CLIMBER_ID = 3;

  //Motor Speeds
  public static final double INTAKE_MOTOR_SPEED = 1;

  //Timeouts
  public static final int DEPLOYED_INTAKE_TIMEOUT = 5;
  public static final int RAISED_INTAKE_TIMEOUT = 3;
  public static final int SHOOTER_TIMEOUT = 1;

  //Limelight Settings
  public static final double CAMERA_HEIGHT = 38.0;
  public static final double TARGET_HEIGHT = 104.0;
  public static final double CAMERA_ANGLE = 18.7;   //cameraAngle is a placeholder value
}
