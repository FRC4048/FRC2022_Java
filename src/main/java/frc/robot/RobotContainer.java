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
import frc.robot.commands.AutoTargetSequence;
import frc.robot.commands.CalibrateTurretEncoderSequence;
import frc.robot.commands.Drive;
import frc.robot.commands.MoveTurretDashboard;
import frc.robot.commands.TurnDegrees;
import frc.robot.commands.ClimberCommands.ManualMoveClimberArm;
import frc.robot.commands.ClimberCommands.ManualMoveClimberWinch;
import frc.robot.commands.ClimberCommands.ToggleClimberSolenoid;
import frc.robot.commands.TurretManualCommand;
import frc.robot.commands.Miscellaneous.SetLEDOff;
import frc.robot.commands.Miscellaneous.SetLEDOn;
import frc.robot.commands.Miscellaneous.SetPipeline;
import frc.robot.commands.ShooterCommands.ElevatorSequence;
import frc.robot.commands.ShooterCommands.ExtendShooterPiston;
import frc.robot.commands.ShooterCommands.ManuallyMoveHood;
import frc.robot.commands.ShooterCommands.MoveHoodDown;
import frc.robot.commands.ShooterCommands.MoveHoodToAngle;
import frc.robot.commands.ShooterCommands.MoveHoodUp;
import frc.robot.commands.ShooterCommands.RetractShooterPiston;
import frc.robot.commands.ShooterCommands.StartShooterMotor;
import frc.robot.commands.ShooterCommands.ShooterParallelSequeunce;
import frc.robot.commands.ShooterCommands.ToggleShooterMotor;
import frc.robot.commands.ShooterCommands.ToggleShooterPiston;
import frc.robot.commands.intakecommands.DeployIntakeCommand;
import frc.robot.commands.intakecommands.DropBallCommand;
import frc.robot.commands.intakecommands.IntakeBallCommand;
import frc.robot.commands.intakecommands.IntakeSequence;
import frc.robot.commands.intakecommands.ManuallyRunIntakeMotor;
import frc.robot.commands.intakecommands.ManuallyToggleIntake;
import frc.robot.commands.intakecommands.RaiseIntakeCommand;
import frc.robot.commands.intakecommands.ReverseIntakeCommand;
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
  private  JoystickButton buttonA = new JoystickButton(xboxController, Constants.XBOX_A_BUTTON);
  private  JoystickButton buttonB = new JoystickButton(xboxController, Constants.XBOX_B_BUTTON);

  private JoystickButton climberButtonA = new JoystickButton(climberController, Constants.XBOX_A_BUTTON);

  private JoystickButton buttonY = new JoystickButton(xboxController, Constants.XBOX_Y_BUTTON);
  
  private JoystickButton buttonX = new JoystickButton(xboxController, Constants.XBOX_X_BUTTON);
  private JoystickButton rightBumper = new JoystickButton(xboxController, Constants.XBOX_RIGHT_BUMPER);
  private JoystickButton leftBumper = new JoystickButton(xboxController, Constants.XBOX_LEFT_BUMPER);
  private Trigger rightTrigger = new Trigger(() -> xboxController.getRightTriggerAxis() > 0.5 );
  private Trigger leftTrigger = new Trigger(() -> xboxController.getRightTriggerAxis() > 0.5 );

  private final LimelightSubsystem limeLightVision = new LimelightSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DriveTrain driveTrain = new DriveTrain();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final PowerDistribution m_PowerDistPanel = new PowerDistribution();
  private final ClimberArmSubsystem climberArmSubsystem = new ClimberArmSubsystem(m_PowerDistPanel);
  private final ClimberWinchSubsystem climberWinchSubsystem = new ClimberWinchSubsystem();
  
  private final Hood hood = new Hood();
  private final TurretSubsystem turretSubsystem= new TurretSubsystem(); 

  public AutoChooser autoChooser = new AutoChooser();

  private final Drive driveCommand = new Drive(driveTrain, () -> joyLeft.getY(), () -> joyRight.getY());
  private final TurretManualCommand turretCommand= new TurretManualCommand(turretSubsystem, () -> xboxController.getLeftX());
  private final ManuallyMoveHood hoodCommand = new ManuallyMoveHood(hood, () -> xboxController.getRightY());

  public boolean canShoot = false;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser.addOptions();
    driveTrain.setDefaultCommand(driveCommand);
    turretSubsystem.setDefaultCommand(turretCommand);
    climberArmSubsystem.setDefaultCommand(new ManualMoveClimberArm(climberArmSubsystem, climberController));
    climberWinchSubsystem.setDefaultCommand(new ManualMoveClimberWinch(climberWinchSubsystem, climberController));

    hood.setDefaultCommand(hoodCommand);

    // Configure the button bindings
    configureButtonBindings();
    autoChooser.initialize();

  }

  public PowerDistribution getPowerDistPanel(){
    return m_PowerDistPanel;
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
    buttonA.whenPressed(new IntakeSequence(intakeSubsystem));
    buttonB.whenPressed(new ManuallyRunIntakeMotor(intakeSubsystem, Constants.INTAKE_MOTOR_SPEED));
    buttonB.whenReleased(new ManuallyRunIntakeMotor(intakeSubsystem, 0));


    climberButtonA.whenPressed(new ToggleClimberSolenoid(climberArmSubsystem));


    buttonY.whenPressed(new ManuallyToggleIntake(intakeSubsystem));


    rightTrigger.whenActive(new ShooterParallelSequeunce(shooterSubsystem, intakeSubsystem));
    leftTrigger.whenActive(new AutoTargetSequence(turretSubsystem, limeLightVision.getLimeLightVision(), hood));
    buttonX.whenPressed(new AutoTargetSequence(turretSubsystem, limeLightVision.getLimeLightVision(), hood));
    rightBumper.whenPressed(new ElevatorSequence(shooterSubsystem, intakeSubsystem));
    leftBumper.whenPressed(new ToggleShooterMotor(shooterSubsystem));
    leftBumper.whenReleased(new ToggleShooterMotor(shooterSubsystem));
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
    return autoChooser.getAutonomousCommand(autoChooser.getPosition() , autoChooser.getAction());
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

      
      SmartShuffleboard.putCommand("Shooter", "Rotate Turret Left", new MoveTurretDashboard(turretSubsystem, MoveTurretDashboard.Direction.LEFT));
      SmartShuffleboard.putCommand("Shooter", "Rotate Turret Right", new MoveTurretDashboard(turretSubsystem, MoveTurretDashboard.Direction.RIGHT));

      SmartShuffleboard.putCommand("Shooter", "Toggle Piston", new ToggleShooterPiston(shooterSubsystem));
      SmartShuffleboard.putCommand("Shooter", "Toggle Shooter Motor", new ToggleShooterMotor(shooterSubsystem));
      SmartShuffleboard.putCommand("Shooter", "Calibrate Enocoder", new CalibrateTurretEncoderSequence(turretSubsystem));
      SmartShuffleboard.putCommand("Shooter", "Start Shooter Motor", new RotateShooterMotor(shooterSubsystem, Constants.SHOOTER_CLOCKWISE_SPEED));

      SmartShuffleboard.putCommand("Shooter", "Extend Piston", new ExtendShooterPiston(shooterSubsystem));
      SmartShuffleboard.putCommand("Shooter", "Retract Piston", new RetractShooterPiston(shooterSubsystem));
      SmartShuffleboard.putCommand("Shooter", "Aim Target", new AutoTargetSequence(turretSubsystem, limeLightVision.getLimeLightVision(), hood));
      SmartShuffleboard.putCommand("Shooter", "Shooter Sequence", new ShooterParallelSequeunce(shooterSubsystem, intakeSubsystem));

      SmartShuffleboard.putCommand("Miscellaneous", "Set LED Off", new SetLEDOff());
      SmartShuffleboard.putCommand("Miscellaneous", "Set LED On", new SetLEDOn());
      SmartShuffleboard.putCommand("Miscellaneous", "Set Pipeline to 0", new SetPipeline(0));
      SmartShuffleboard.putCommand("Miscellaneous", "Set Pipeline to 1", new SetPipeline(1));
      SmartShuffleboard.putCommand("Turn", "Turn Degrees", new TurnDegrees(driveTrain, 90));


      SmartShuffleboard.putCommand("Hood", "Move Hood Down", new MoveHoodDown(hood));
      SmartShuffleboard.putCommand("Hood", "Move Hood Up", new MoveHoodUp(hood));

 
    }
  }
} 
