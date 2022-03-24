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
  public static final int CONTROLLER_CLIMBER_ID = 3;

  //Motor Speeds
  public static final double INTAKE_MOTOR_SPEED = 1;

  //Timeouts
  public static final int DEPLOYED_INTAKE_TIMEOUT = 6;
  public static final int RAISED_INTAKE_TIMEOUT = 3;
  public static final int SHOOTER_TIMEOUT = 4;
  public static final double HOOD_MOTOR_TIMEOUT = 5;
  public static final double TURRETSPIN_TIMEOUT = 5;
  public static final double MOVEDISTANCE_TIMEOUT = 10;
  public static final double INTAKE_MOTOR_TIMEOUT = 10;
  public static final double TURRETSPIN_COMMAND_TIMEOUT = 2;
  public static final double HOOD_TARGET_TIMEOUT = 1;
  public static final double EXTEND_WINCH_TIMEOUT = 3;

  //Limelight Settings
  public static final double CAMERA_HEIGHT = 38.5;
  public static final double TARGET_HEIGHT = 104.0;
  public static final int LIMELIGHT_TARGET_DETECTION = 1;
  public static final int LIMELIGHT_STREAMING = 0;
  public static final double CAMERA_ANGLE = 29.0;

  //Hood
  public static final double HOOD_MOTOR_SPEED = 0.9;
  public static final double HOOD_AUTO_MOTOR_SPEED = 1;
  public static final double HOOD_AUTO_LIMIT = 2;
  public static final double HOOD_JOYSTICK_THRESHOLD = 0.2;
  public static final double HOOD_RANGE_OF_MOTION = 180; //temp value
  public static final double HOOD_STARTING_POINT = 0; //temp value
  public static final double HOOD_ERROR_THRESHOLD = .5;


  //Climber
  public static final double CLIMBER_V_LIMIT = 26;
  public static final double CLIMBER_ARM_V_TIMEOUT = 0.3;
  public static final double CLIMBER_SLOW_ARM_RATE = 0.75;
  public static final double CLIMBER_SLOW_WINCH_RATE = 0.75;
  public static final double CLIMBER_ARM_SPEED = .3;
  public static final double CLIMBER_WINCH_SPEED = .5;
  public static final double CLIMBER_MIN_ARM_SPEED = 0.1;
  public static final double CLIMBER_MIN_WINCH_SPEED = 0.1;
  public static final double CLIMBER_TIMEOUT = 10;
  public static final double CLIMBER_MAX_ENCODER_DIFF = 1000;
  public static final double CLIMBER_ARM_TIMEOUT = 10;
  public static final double CLIMBER_WINCH_TIMEOUT = 10;
  public static final double CLIMBER_DEAD_ZONE = 0.5;

  //Shooter
  public static final double SHOOTER_SPEED = 0.5;
  public static final double PISTON_DELAY = 0.5;
  public static final double SHOOTER_SPINUP_DELAY = 1;
  public static final double SHOOTER_PISTON_WAIT = .7;
  public static final double SHOOTER_RPM = 12000;

  //Shooter PID
  public static final double SHOOTER_PID_P = 6e-5;
  public static final double SHOOTER_PID_I = 0;
  public static final double SHOOTER_PID_D = 6e-6;
  public static final double SHOOTER_PID_IZ = 0;
  public static final double SHOOTER_PID_FF = 0.000015;
  public static final double SHOOTER_MAX_OUTPUT = 1;
  public static final double SHOOTER_MIN_OUTPUT = -1;

  //Drive
  public static final double CONTROLLER_DEAD_ZONE = 0.05;

  // Turret
  public static final double TURRET_CLOCKWISE_SPEED = .5;
  public static final double TURRET_COUNTERCLOCKWISE_SPEED = -.5;
  // Intake
  public static final double INTAKE_BUFFER = 1.0;
  public static final double INTAKE_BALL_DETECTIONS_THRESHOLD = 5;

  //Turret
  public static final double TURRETSPIN_SCALEFACTOR = 0.35;
  public static final int TURRET_AUTO_BAD_READINGS_TRESHOLD = 50;
  public static final double TURRET_AUTO_ALIGN_TRESHOLD = 0.2; //change when testing
  public static final double TURRET_AUTO_TIMEOUT = 5;
  public static final double TURRET_SPEED = 0.7;
  public static final double TURRET_MIN_SPEED = 0.1;
  public static final double TURRET_MAX_DIFFERENCE = 500;
  public static final double TURRET_SWEEP_SPEED = 0.4;

  // SHOOTER MATH
  public static final int HOOD_MARGIN_OF_ERROR = 1;
}
