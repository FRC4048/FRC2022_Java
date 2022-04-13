// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;

public class ExtendWinchForTimeout extends CommandBase {
  /** Creates a new PullOffBar. */
  ClimberWinchSubsystem climberWinchSubsystem; 
  private double initTime;
  public ExtendWinchForTimeout(ClimberWinchSubsystem climberWinchSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberWinchSubsystem = climberWinchSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberWinchSubsystem.setSpeed(Constants.CLIMBER_WINCH_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberWinchSubsystem.stopLeftWinch();
    climberWinchSubsystem.stopRightWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Timer.getFPGATimestamp() - initTime) >= Constants.CLIMBER_PULL_OFF_TIMEOUT);
  }
}
