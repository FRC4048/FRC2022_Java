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
import frc.robot.commands.intakecommands.IntakeSequence;
import frc.robot.commands.ShooterCommands.TogglePiston;
import frc.robot.commands.ShooterCommands.ToggleShooterMotor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.SmartShuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autonomousCommand.CrossLine;
import frc.robot.commands.autonomousCommand.DoNothing;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  private  JoystickButton buttonA = new JoystickButton(xboxController, Constants.XBOX_A_BUTTON);

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DriveTrain driveTrain = new DriveTrain();
  private final Shooter shooterSubsystem = new Shooter();
  private final PowerDistribution m_PowerDistPanel = new PowerDistribution();

  public AutoChooser autoChooser = new AutoChooser();

  private final Drive driveCommand = new Drive(driveTrain, () -> joyLeft.getY(), () -> joyRight.getY());

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
    buttonA.whenPressed(new IntakeSequence(intakeSubsystem));
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return intakeSubsystem;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(AutoChooser.AutoCommand autoOption) {
    // An ExampleCommand will run in autonomous
    Command autoCommand; 
   
    switch(autoOption){
    case DO_NOTHING:
        autoCommand = new DoNothing();
        break;
    case CROSS_LINE:
        autoCommand= new CrossLine();
        break;
    }
    
    return autoCommand;
  }

  public void installCommandsOnShuffleboard() {
    if (Constants.ENABLE_DEBUG) {
      SmartShuffleboard.putCommand("Intake", "Deploy Intake", new DeployIntakeCommand(getIntakeSubsystem()));
      SmartShuffleboard.putCommand("Intake", "Raise Intake", new RaiseIntakeCommand(getIntakeSubsystem()));
      SmartShuffleboard.putCommand("Intake", "Intake Ball", new IntakeBallCommand(getIntakeSubsystem()));
      SmartShuffleboard.putCommand("Intake", "Drop Ball", new DropBallCommand(getIntakeSubsystem()));

      SmartShuffleboard.putCommand("Shooter", "Toggle Piston", new TogglePiston(shooterSubsystem));
      SmartShuffleboard.putCommand("Shooter", "Toggle Shooter Motor", new ToggleShooterMotor(shooterSubsystem));
    }
  }
} 
