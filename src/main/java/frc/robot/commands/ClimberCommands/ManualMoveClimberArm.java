// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberArmSubsystem;

public class ManualMoveClimberArm extends CommandBase {
  /** Creates a new ManualMoveClimbArm. */
  private ClimberArmSubsystem climberArmSubsystem;
  private XboxController climberController;
  
  public ManualMoveClimberArm(ClimberArmSubsystem climberArmSubsystem, XboxController climberController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberArmSubsystem = climberArmSubsystem;
    this.climberController = climberController;
    addRequirements(climberArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climberController.getLeftY() > 0.75) {
      climberArmSubsystem.setSpeed(Constants.CLIMBER_WINCH_SPEED);
    } else if (climberController.getLeftY() < 0.75) {
      climberArmSubsystem.setSpeed(-Constants.CLIMBER_WINCH_SPEED);
    } else {
      climberArmSubsystem.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
