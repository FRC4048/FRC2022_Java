// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoTurnDegrees;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.SmartShuffleboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private static Joystick joyLeft = new Joystick(0);
  private static Joystick joyRight = new Joystick(1);

  private final DriveTrain drivetrain = new DriveTrain();

  private final Drive command = new Drive(drivetrain, () -> joyLeft.getY(), () -> joyRight.getY());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    drivetrain.setDefaultCommand(command);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  public void installCommandsOnShuffleboard () {
    SmartShuffleboard.putCommand("Drive", "Turn 30 Degrees", new AutoTurnDegrees(drivetrain, 30));
    SmartShuffleboard.putCommand("Drive", "Turn -30 Degrees", new AutoTurnDegrees(drivetrain, -30));
    SmartShuffleboard.putCommand("Drive", "Turn 90 Degrees", new AutoTurnDegrees(drivetrain, 90));
    SmartShuffleboard.putCommand("Drive", "Turn 120 Degrees", new AutoTurnDegrees(drivetrain, 120));
    SmartShuffleboard.putCommand("Drive", "Turn 150 Degrees", new AutoTurnDegrees(drivetrain, 150));
    SmartShuffleboard.putCommand("Drive", "Turn 180 Degrees", new AutoTurnDegrees(drivetrain, 180));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return command;
  }

  public DriveTrain getDriveTrain() {
    return drivetrain;
  }
}
