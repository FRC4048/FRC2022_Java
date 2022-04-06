// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LogError;
import frc.robot.commands.ToggleBlockerPiston;
import frc.robot.commands.Autonomous.AutoSetShootingPosition;
import frc.robot.commands.Autonomous.AutoSetTurretPosition;
import frc.robot.commands.Autonomous.CrossTheLineSequence;
import frc.robot.commands.Autonomous.DoNothingSequence;
import frc.robot.commands.Autonomous.OneShotSequenceMiddle;
import frc.robot.commands.Autonomous.TwoShotSequenceLeft;
import frc.robot.commands.Autonomous.TwoShotSequenceRight;
import frc.robot.commands.ClimberCommands.AutoMoveClimberArm;
import frc.robot.commands.ClimberCommands.AutoMoveClimberArm.ClimberDirection;
import frc.robot.commands.ClimberCommands.CloseStaticHooks;
import frc.robot.commands.ClimberCommands.InitialClimbSequence;
import frc.robot.commands.ClimberCommands.ClimberExtendSequence;
import frc.robot.commands.ClimberCommands.ManualMoveClimberArm;
import frc.robot.commands.ClimberCommands.ManualMoveClimberWinch;
import frc.robot.commands.ClimberCommands.MoveClimberToNextBar;
import frc.robot.commands.ClimberCommands.OpenStaticHooks;
import frc.robot.commands.ClimberCommands.RetractClimberSequence;
import frc.robot.commands.ClimberCommands.Traversal;
import frc.robot.commands.DriveCommands.AutoTurnDegrees;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.commands.DriveCommands.TurnDegrees;
import frc.robot.commands.HoodCommands.HoodAutoCommand;
import frc.robot.commands.HoodCommands.HoodContinousTarget;
import frc.robot.commands.HoodCommands.MoveHoodDown;
import frc.robot.commands.HoodCommands.MoveHoodToAngle;
import frc.robot.commands.HoodCommands.MoveHoodUp;
import frc.robot.commands.IntakeCommand.DeployIntakeCommand;
import frc.robot.commands.IntakeCommand.DropBallCommand;
import frc.robot.commands.IntakeCommand.DropBallCommandManual;
import frc.robot.commands.IntakeCommand.IntakeBallCommand;
import frc.robot.commands.IntakeCommand.IntakeSequence;
import frc.robot.commands.IntakeCommand.ManuallyRunIntakeMotor;
import frc.robot.commands.IntakeCommand.ManuallyToggleIntake;
import frc.robot.commands.IntakeCommand.RaiseIntakeCommand;
import frc.robot.commands.Miscellaneous.CancelAll;
import frc.robot.commands.Miscellaneous.SetLEDOff;
import frc.robot.commands.Miscellaneous.SetLEDOn;
import frc.robot.commands.Miscellaneous.SetPipeline;
import frc.robot.commands.ShooterCommands.AutoTargetSequence;
import frc.robot.commands.ShooterCommands.ExtendShooterPiston;
import frc.robot.commands.ShooterCommands.LaunchpadSetPoint;
import frc.robot.commands.ShooterCommands.RetractShooterPiston;
import frc.robot.commands.ShooterCommands.RunShooterMotor;
import frc.robot.commands.ShooterCommands.SetShooterMotor;
import frc.robot.commands.ShooterCommands.ShooterSequeunce;
import frc.robot.commands.ShooterCommands.TarmacSetPoint;
import frc.robot.commands.ShooterCommands.ToggleShooterMotor;
import frc.robot.commands.ShooterCommands.ToggleShooterPiston;
import frc.robot.commands.ShooterCommands.VisionAutoShooter;
import frc.robot.commands.ShooterCommands.WaitForRPM;
import frc.robot.commands.TurretCommands.CalibrateTurretEncoderSequence;
import frc.robot.commands.TurretCommands.MoveTurretDashboard;
import frc.robot.commands.TurretCommands.RunTurretUntilLimitSwitch;
import frc.robot.commands.TurretCommands.RunTurretUntilTarget;
import frc.robot.commands.TurretCommands.ToggleTargetState;
import frc.robot.commands.TurretCommands.TurretContinousTarget;
import frc.robot.commands.TurretCommands.TurretSweepSequence;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Climber.ClimberArmSubsystem;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.limelight.LimeLightVision;
import frc.robot.utils.logging.LogCommandWrapper;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */



public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  private static Joystick joyLeft = new Joystick(Constants.LEFT_JOYSTICK_ID);
  private static Joystick joyRight = new Joystick(Constants.RIGHT_JOYSTICK_ID);

  private XboxController xboxController = new XboxController(Constants.CONTROLLER_ID);
  private XboxController climberController = new XboxController(Constants.CONTROLLER_CLIMBER_ID);
  private JoystickButton buttonA = new JoystickButton(xboxController, Constants.XBOX_A_BUTTON);
  private JoystickButton buttonB = new JoystickButton(xboxController, Constants.XBOX_B_BUTTON);
  

  private JoystickButton climberAButton = new JoystickButton(climberController, Constants.XBOX_A_BUTTON);
  private JoystickButton climberBButton = new JoystickButton(climberController, Constants.XBOX_B_BUTTON);
  private JoystickButton staticLock = new JoystickButton(climberController, Constants.XBOX_LEFT_BUMPER);
  private JoystickButton staticUnlock = new JoystickButton(climberController, Constants.XBOX_RIGHT_BUMPER);
  private JoystickButton climberYButton = new JoystickButton(climberController, Constants.XBOX_Y_BUTTON);
  private JoystickButton climberXButton = new JoystickButton(climberController, Constants.XBOX_X_BUTTON);
  private JoystickButton climberBackButton = new JoystickButton(climberController, Constants.XBOX_BACK_BUTTON);

  private JoystickButton buttonY = new JoystickButton(xboxController, Constants.XBOX_Y_BUTTON);
  private JoystickButton buttonX = new JoystickButton(xboxController, Constants.XBOX_X_BUTTON);

  private JoystickButton rightBumper = new JoystickButton(xboxController, Constants.XBOX_RIGHT_BUMPER);
  private JoystickButton leftBumper = new JoystickButton(xboxController, Constants.XBOX_LEFT_BUMPER);
  private JoystickButton startButton = new JoystickButton(xboxController, Constants.XBOX_START_BUTTON);
  private Trigger rightTrigger = new Trigger(() -> xboxController.getRightTriggerAxis() > 0.5 );
  private Trigger leftTrigger = new Trigger(() -> xboxController.getLeftTriggerAxis() > 0.5 );
  private JoystickButton backButton = new JoystickButton(xboxController, Constants.XBOX_BACK_BUTTON);

  private final LimelightSubsystem limeLightVision = new LimelightSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DriveTrain driveTrain = new DriveTrain();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final PowerDistribution m_PowerDistPanel = new PowerDistribution();
  private final ClimberArmSubsystem climberArmSubsystem = new ClimberArmSubsystem(m_PowerDistPanel);
  private final ClimberWinchSubsystem climberWinchSubsystem = new ClimberWinchSubsystem(m_PowerDistPanel);
  
  private final Hood hood = new Hood();
  private final TurretSubsystem turretSubsystem= new TurretSubsystem(); 

  public AutoChooser autoChooser = new AutoChooser(intakeSubsystem, driveTrain, shooterSubsystem, turretSubsystem, limeLightVision.getLimeLightVision(), hood);

  private final Drive driveCommand = new Drive(driveTrain, () -> joyLeft.getY(), () -> joyRight.getY());
  private final TurretContinousTarget turretCommand= new TurretContinousTarget(turretSubsystem, limeLightVision.getLimeLightVision(), () -> -xboxController.getLeftX());
  private final HoodContinousTarget hoodCommand = new HoodContinousTarget(hood, () -> xboxController.getRightY(), limeLightVision.getLimeLightVision());

  public boolean canShoot = false;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser.addOptions();
    driveTrain.setDefaultCommand(driveCommand);
    turretSubsystem.setDefaultCommand(turretCommand);
    climberArmSubsystem.setDefaultCommand(new ManualMoveClimberArm(climberArmSubsystem, climberController));
    climberWinchSubsystem.setDefaultCommand(new ManualMoveClimberWinch(climberWinchSubsystem, climberController));
    shooterSubsystem.setDefaultCommand(new RunShooterMotor(shooterSubsystem));

    hood.setDefaultCommand(hoodCommand);

    // Configure the button bindings
    configureButtonBindings();
    autoChooser.initialize();

  }

  public PowerDistribution getPowerDistPanel(){
    return m_PowerDistPanel;
  }

  public TurretSubsystem getTurretSubsystem() {
    return turretSubsystem;
  }

  public Hood getHood() {
    return hood;
  }

  /*
  public void doRumble() {
    joyLeft.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
		joyRight.setRumble(GenericHID.RumbleType.kRightRumble, 1);
  }

  public void stopRumble() {
    joyLeft.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    joyRight.setRumble(GenericHID.RumbleType.kRightRumble, 0);
  }
  */



  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   
    SmartShuffleboard.putCommand("Shooter", "Toggle Piston", new ToggleShooterPiston(shooterSubsystem));
    SmartShuffleboard.putCommand("Shooter", "Extend Piston", new ExtendShooterPiston(shooterSubsystem));
    SmartShuffleboard.putCommand("Shooter", "Retract Piston", new RetractShooterPiston(shooterSubsystem));
    SmartShuffleboard.putCommand("Shooter", "Aim Target", new AutoTargetSequence(turretSubsystem, limeLightVision.getLimeLightVision(), hood));

    // Climber Controls
    climberAButton.whenPressed(new LogCommandWrapper(new ClimberExtendSequence(climberArmSubsystem, climberWinchSubsystem, turretSubsystem, limeLightVision.getLimeLightVision())));
    //climberAButton.whenPressed(new LogCommandWrapper(new InitialClimbSequence(climberArmSubsystem, climberWinchSubsystem, turretSubsystem, limeLightVision.getLimeLightVision(), climberController)));
    climberBButton.whenPressed(new LogCommandWrapper(new RetractClimberSequence(climberWinchSubsystem, climberArmSubsystem)));
    staticLock.whenPressed(new LogCommandWrapper(new CloseStaticHooks(climberWinchSubsystem)));
    staticUnlock.whenPressed(new LogCommandWrapper(new OpenStaticHooks(climberWinchSubsystem)));
    climberYButton.whenPressed(new LogCommandWrapper(new MoveClimberToNextBar(climberArmSubsystem, climberWinchSubsystem, climberController)));
    climberXButton.whenPressed(new LogCommandWrapper(new InitialClimbSequence(climberArmSubsystem, climberWinchSubsystem, turretSubsystem, limeLightVision.getLimeLightVision(), climberController)));
    climberBackButton.whenPressed(new LogCommandWrapper(new Traversal(climberArmSubsystem, climberWinchSubsystem, turretSubsystem, limeLightVision.getLimeLightVision(), climberController)));
    
    buttonA.whenPressed(new LogCommandWrapper(new IntakeSequence(intakeSubsystem)));
    buttonB.whenPressed(new LogCommandWrapper(new ManuallyRunIntakeMotor(intakeSubsystem, Constants.INTAKE_MOTOR_SPEED)));
    buttonB.whenReleased(new LogCommandWrapper(new ManuallyRunIntakeMotor(intakeSubsystem, 0)));
    buttonY.whenPressed(new LogCommandWrapper(new ManuallyToggleIntake(intakeSubsystem)));
    buttonX.whenPressed(new LogCommandWrapper(new DropBallCommandManual(intakeSubsystem, 0.8)));

    rightTrigger.whenActive(new LogCommandWrapper(new ShooterSequeunce(shooterSubsystem, limeLightVision.getLimeLightVision())));
    leftTrigger.whenActive(new LogCommandWrapper(new ToggleTargetState()));
    leftBumper.whenPressed(new LogCommandWrapper(new TarmacSetPoint(hood, shooterSubsystem)));
    rightBumper.whenPressed(new LogCommandWrapper(new LaunchpadSetPoint(hood, shooterSubsystem)));
    startButton.whenPressed(new LogError());
    backButton.whenPressed(new LogCommandWrapper(new CancelAll(intakeSubsystem, shooterSubsystem)));

  }

  public IntakeSubsystem getIntakeSubsystem() {
    return intakeSubsystem;
  }

  public LimeLightVision getLimeLight() {
    return limeLightVision.getLimeLightVision();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return autoChooser.getAutonomousCommand(autoChooser.getAction());
  }

  public void installDriverShuffleboard() {
    SmartShuffleboard.putCommand("Driver", "Camera Detection", new SetPipeline(Constants.LIMELIGHT_TARGET_DETECTION));
    SmartShuffleboard.putCommand("Driver", "Camera Streaming", new SetPipeline(Constants.LIMELIGHT_STREAMING));
  }
  

  public void installCommandsOnShuffleboard() {
    if (Constants.ENABLE_DEBUG) {
      SmartShuffleboard.putCommand("Intake", "Deploy Intake", new DeployIntakeCommand(getIntakeSubsystem()));
      SmartShuffleboard.putCommand("Intake", "Raise Intake", new RaiseIntakeCommand(getIntakeSubsystem()));
      SmartShuffleboard.putCommand("Intake", "Intake Ball", new IntakeBallCommand(getIntakeSubsystem()));
      SmartShuffleboard.putCommand("Intake", "Drop Ball", new DropBallCommand(getIntakeSubsystem()));

      SmartShuffleboard.putCommand("Climber", "Extend Arm", new AutoMoveClimberArm(climberArmSubsystem, ClimberDirection.EXTEND));
      SmartShuffleboard.putCommand("Climber", "Retract Arm", new AutoMoveClimberArm(climberArmSubsystem, ClimberDirection.RETRACT));
      SmartShuffleboard.putCommand("Climber", "Transfer", new MoveClimberToNextBar(climberArmSubsystem, climberWinchSubsystem, climberController));

      SmartShuffleboard.putCommand("Shooter", "Rotate Turret Left", new MoveTurretDashboard(turretSubsystem, MoveTurretDashboard.Direction.LEFT));
      SmartShuffleboard.putCommand("Shooter", "Rotate Turret Right", new MoveTurretDashboard(turretSubsystem, MoveTurretDashboard.Direction.RIGHT));
      SmartShuffleboard.putCommand("Shooter", "Turret Sweep", new TurretSweepSequence(turretSubsystem, limeLightVision.getLimeLightVision()));
      SmartShuffleboard.putCommand("Shooter", "Run Turret Until Target", new RunTurretUntilTarget(turretSubsystem, limeLightVision.getLimeLightVision(), Constants.TURRET_SWEEP_SPEED));

      SmartShuffleboard.putCommand("Shooter", "Toggle Piston", new ToggleShooterPiston(shooterSubsystem));
      SmartShuffleboard.putCommand("Shooter", "Toggle Shooter Motor", new ToggleShooterMotor(shooterSubsystem, Constants.SHOOTER_RPM));
      SmartShuffleboard.putCommand("Shooter", "Calibrate Enocoder", new CalibrateTurretEncoderSequence(turretSubsystem));
      SmartShuffleboard.putCommand("Shooter", "Shooter Motor 12000", new SetShooterMotor(shooterSubsystem, Constants.SHOOTER_RPM));
      SmartShuffleboard.putCommand("Shooter", "Shooter Motor 12600", new SetShooterMotor(shooterSubsystem, 12600));
      SmartShuffleboard.putCommand("Shooter", "Shooter Motor 12900", new SetShooterMotor(shooterSubsystem, 12900));
      SmartShuffleboard.putCommand("Shooter", "Shooter Motor 0", new SetShooterMotor(shooterSubsystem, 0));
      SmartShuffleboard.putCommand("Shooter", "Wait For RPM", new WaitForRPM(shooterSubsystem));

      SmartShuffleboard.putCommand("Shooter", "Extend Piston", new ExtendShooterPiston(shooterSubsystem));
      SmartShuffleboard.putCommand("Shooter", "Retract Piston", new RetractShooterPiston(shooterSubsystem));
      SmartShuffleboard.putCommand("Shooter", "Aim Target", new AutoTargetSequence(turretSubsystem, limeLightVision.getLimeLightVision(), hood));
      SmartShuffleboard.putCommand("Shooter", "Shooter Sequence", new ShooterSequeunce(shooterSubsystem, limeLightVision.getLimeLightVision()));
      SmartShuffleboard.putCommand("Block", "Extend Block", new ToggleBlockerPiston(shooterSubsystem, true));
      SmartShuffleboard.putCommand("Block", "Retract Block", new ToggleBlockerPiston(shooterSubsystem, false));
      SmartShuffleboard.putCommand("Shooter", "Shoot on Vision", new VisionAutoShooter(limeLightVision.getLimeLightVision(), shooterSubsystem));

      SmartShuffleboard.putCommand("Miscellaneous", "Set LED Off", new SetLEDOff());
      SmartShuffleboard.putCommand("Miscellaneous", "Set LED On", new SetLEDOn());
      SmartShuffleboard.putCommand("Miscellaneous", "Set Pipeline to 0", new SetPipeline(0));
      SmartShuffleboard.putCommand("Miscellaneous", "Set Pipeline to 1", new SetPipeline(1));
      SmartShuffleboard.putCommand("Turn", "Turn Degrees", new TurnDegrees(driveTrain, 90));
      SmartShuffleboard.putCommand("Drive", "Turn 30 Degrees", new AutoTurnDegrees(driveTrain, 30));
      SmartShuffleboard.putCommand("Drive", "Turn -30 Degrees", new AutoTurnDegrees(driveTrain, -30));
      SmartShuffleboard.putCommand("Drive", "Turn 90 Degrees", new AutoTurnDegrees(driveTrain, 90));
      SmartShuffleboard.putCommand("Drive", "Turn 120 Degrees", new AutoTurnDegrees(driveTrain, 120));
      SmartShuffleboard.putCommand("Drive", "Turn 150 Degrees", new AutoTurnDegrees(driveTrain, 150));
      SmartShuffleboard.putCommand("Drive", "Turn 180 Degrees", new AutoTurnDegrees(driveTrain, 180));


      SmartShuffleboard.putCommand("Hood", "Move Hood Down", new MoveHoodDown(hood));
      SmartShuffleboard.putCommand("Hood", "Move Hood Up", new MoveHoodUp(hood));

      SmartShuffleboard.putCommand("Autonomous", "Two Shot Left", new TwoShotSequenceLeft(turretSubsystem, Constants.AUTO_TURRET_SPEED,  intakeSubsystem, driveTrain, Constants.AUTO_MOVE_SPEED, Constants.AUTO_DISTANCE_INCHES, shooterSubsystem, limeLightVision.getLimeLightVision(), hood));
      SmartShuffleboard.putCommand("Autonomous", "Two Shot Right", new TwoShotSequenceRight(turretSubsystem, Constants.AUTO_TURRET_SPEED,  intakeSubsystem, driveTrain, Constants.AUTO_MOVE_SPEED, Constants.AUTO_DISTANCE_INCHES, shooterSubsystem, limeLightVision.getLimeLightVision(), hood));
      SmartShuffleboard.putCommand("Autonomous", "Two Shot Middle", new OneShotSequenceMiddle(turretSubsystem, intakeSubsystem, driveTrain, shooterSubsystem, limeLightVision.getLimeLightVision(), hood, Constants.AUTO_MOVE_SPEED, Constants.AUTO_DISTANCE_INCHES));
      SmartShuffleboard.putCommand("Autonomous", "Cross Line", new CrossTheLineSequence(driveTrain));
      SmartShuffleboard.putCommand("Autonomous", "Do Nothing", new DoNothingSequence());
      SmartShuffleboard.putCommand("Autonomous", "Turret Reset", new AutoSetShootingPosition(turretSubsystem, Constants.AUTO_TURRET_SPEED, Constants.AUTO_TURRET_CENTER_ANGLE));
      SmartShuffleboard.putCommand("Autonomous", "Run Turret To Switch", new RunTurretUntilLimitSwitch(turretSubsystem));
      SmartShuffleboard.putCommand("Autonomous", "Set Turret Middle", new AutoSetTurretPosition(turretSubsystem, Constants.AUTO_TURRET_SPEED, Constants.AUTO_TURRET_CENTER_ANGLE));


      SmartShuffleboard.putCommand("Hood", "Move to 106", new MoveHoodToAngle(hood, 106.0));
      SmartShuffleboard.putCommand("Hood", "Move to 120", new MoveHoodToAngle(hood, 120.0));
      SmartShuffleboard.putCommand("Hood", "Move to 130", new MoveHoodToAngle(hood, 130.0));

      SmartShuffleboard.putCommand("Hood", "Auto", new HoodAutoCommand(hood, limeLightVision.getLimeLightVision()));
    }
  }
} 
