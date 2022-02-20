// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public interface Constants2022TheTB {

  //Global Constant
  public static final boolean ENABLE_DEBUG = true;

  //CAN ID
  public static final int PDP_CAN_ID = 0;
  public static final int DRIVE_LEFT1_ID = 41;
  public static final int DRIVE_LEFT2_ID = 42;
  public static final int DRIVE_RIGHT1_ID = 44;
  public static final int DRIVE_RIGHT2_ID = 45;
  public static final int PCM_CAN_ID = 20;
  public static final int SHOOTER_MOTOR_ID = 42;
  public static final int INTAKE_MOTOR_ID = 10;

  //PDP
  public static final int PDP_DRIVE_L1 = 13;
  public static final int PDP_DRIVE_L2 = 15;
  public static final int PDP_DRIVE_R1 = 0;
  public static final int PDP_DRIVE_R2 = 1;

  //DIO
  public static final int INTAKE_SENSOR_ID = 0;

  //Relay

  //PCM
  public static final int SHOOTER_PISTON_ID = 6;
  public static final int INTAKE_SOLENOID_1 = 0;
  public static final int INTAKE_SOLENOID_2 = 1;

  //DRIVETRAIN CONSTANTS

  //SHOOTER CONSTANTS

  // Intake
  public static final double INTAKE_MOTOR_SPEED = 1.0;
  public static final int RAISED_INTAKE_TIMEOUT = 3;
  public static final int DEPLOYED_INTAKE_TIMEOUT = 5;
  public static final double INTAKE_BUFFER = 1.0;

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
  double cameraHeight = 38.0;
  double targetHeight = 104.0;
  //cameraAngle is a placeholdervalue
  double cameraAngle = 18.7;
}
