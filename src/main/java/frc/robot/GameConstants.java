package frc.robot;

public interface GameConstants {
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
  public static final int SHOOTER_TIMEOUT = 15;

  //Limelight Settings
  public static final double CAMERA_HEIGHT = 38.0;
  public static final double TARGET_HEIGHT = 104.0;

  //cameraAngle is a placeholder value
  public static final double CAMERA_ANGLE = 18.7;

  //Hood
  public static final double HOOD_MOTOR_SPEED = 0.75;
  public static final double HOOD_AUTO_MOTOR_SPEED = 0.5;
  public static final double HOOD_AUTO_LIMIT = 2;
  public static final double HOOD_JOYSTICK_THRESHOLD = 0.2;

  //Climber
  public static final double CLIMBER_V_LIMIT = 26;
  public static final double CLIMBER_ARM_V_TIMEOUT = 0.3;

  //Shooter
  public static final double SHOOTER_CLOCKWISE_SPEED = 0.5;
  public static final double SHOOTER_COUNTERCLOCKWISE_SPEED = -0.5;
  public static final double PISTON_DELAY = 0.5;
  public static final double SHOOTER_SPINUP_DELAY = 1;
  
  // Intake
  public static final double INTAKE_BUFFER = 1.0;

  //Turret
  public static final double TURRETSPIN_SCALEFACTOR = 0.5;
}
