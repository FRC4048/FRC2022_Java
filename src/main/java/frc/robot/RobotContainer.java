// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.PowerDistribution;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.AutoChooser.AutoCommand;
import frc.robot.commands.intakecommands.DeployIntakeCommand;
import frc.robot.commands.intakecommands.DropBallCommand;
import frc.robot.commands.intakecommands.IntakeBallCommand;
import frc.robot.commands.intakecommands.RaiseIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.Drive;
import frc.robot.commands.Miscellaneous.SetLEDOff;
import frc.robot.commands.Miscellaneous.SetLEDOn;
import frc.robot.commands.Miscellaneous.SetPipeline0;
import frc.robot.commands.Miscellaneous.SetPipeline1;
import frc.robot.commands.ShooterCommands.ToggleShooterPiston;
import frc.robot.commands.ShooterCommands.RetractShooterPiston;
import frc.robot.commands.ShooterCommands.ExtendShooterPiston;
import frc.robot.commands.ShooterCommands.ToggleShooterMotor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.limelight.LimeLightVision;
import edu.wpi.first.wpilibj2.command.Command;

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
  private static Joystick controller = new Joystick(Constants.CONTROLLER_ID);
  private XboxController xboxController = new XboxController(Constants.CONTROLLER_ID);

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DriveTrain driveTrain = new DriveTrain();
  private final Shooter shooterSubsystem = new Shooter();
  private final PowerDistribution m_PowerDistPanel = new PowerDistribution();
  
  public AutoChooser autoChooser = new AutoChooser();

  private final Drive driveCommand = new Drive(driveTrain, () -> joyLeft.getY(), () -> joyRight.getY());
  private final LimeLightVision limeLight = new LimeLightVision(20.9, 80.8, 20.7);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser.addOptions();
    driveTrain.setDefaultCommand(new Drive(driveTrain, () -> joyLeft.getY(), () -> joyRight.getY()));

    // Configure the button bindings
    configureButtonBindings();
    autoChooser.initialize();

  
  }

  public PowerDistribution getPowerDistPanel(){
    return m_PowerDistPanel;
  }

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
      SmartShuffleboard.putCommand("Shooter", "Toggle Shooter Motor", new ToggleShooterMotor(shooterSubsystem));
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return intakeSubsystem;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return driveCommand;
  }

  public void installCommandsOnShuffleboard() {
    if (Constants.ENABLE_DEBUG) {
      SmartShuffleboard.putCommand("Intake", "Deploy Intake", new DeployIntakeCommand(getIntakeSubsystem()));
      SmartShuffleboard.putCommand("Intake", "Raise Intake", new RaiseIntakeCommand(getIntakeSubsystem()));
      SmartShuffleboard.putCommand("Intake", "Intake Ball", new IntakeBallCommand(getIntakeSubsystem()));
      SmartShuffleboard.putCommand("Intake", "Drop Ball", new DropBallCommand(getIntakeSubsystem()));

      

      SmartShuffleboard.putCommand("Miscellaneous", "Set LED Off", new SetLEDOff());
      SmartShuffleboard.putCommand("Miscellaneous", "Set LED On", new SetLEDOn());
      SmartShuffleboard.putCommand("Miscellaneous", "Set Pipeline 0", new SetPipeline0());
      SmartShuffleboard.putCommand("Miscellaneous", "Set Pipeline 1", new SetPipeline1());
    }
  }
} 
