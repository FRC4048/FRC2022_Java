// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberArmSubsystem;

public class ManualMoveClimberArm extends CommandBase {
  /** Creates a new ManualMoveClimberArm. */
  private ClimberArmSubsystem climberArmSubsystem;
  private XboxController xboxController;
  public ManualMoveClimberArm(ClimberArmSubsystem climberArmSubsystem, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberArmSubsystem = climberArmSubsystem;
    this.xboxController = xboxController;
    addRequirements(climberArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climberArmSubsystem.getLeftArmVoltage() < 20) {
      climberArmSubsystem.setLeftArmSpeed(xboxController.getLeftY() * Constants.CLIMBER_ARM_SPEED);
    }
    if (climberArmSubsystem.getRightArmVoltage() < 20) {
      climberArmSubsystem.setRightArmSpeed(xboxController.getLeftY() * Constants.CLIMBER_ARM_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberArmSubsystem.stopRightArm();
    climberArmSubsystem.stopLeftArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
