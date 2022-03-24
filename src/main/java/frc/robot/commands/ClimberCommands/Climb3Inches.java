// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;

public class Climb3Inches extends CommandBase {
  /** Creates a new StartClimb. */
  private ClimberWinchSubsystem climberWinchSubsystem;
  private double leftStart;
  private double rightStart;
  private double startTime;

  public Climb3Inches(ClimberWinchSubsystem climberWinchSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberWinchSubsystem = climberWinchSubsystem;
    addRequirements(climberWinchSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftStart = climberWinchSubsystem.getLeftEncoder();
    rightStart = climberWinchSubsystem.getRightEncoder();
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberWinchSubsystem.setLeftWinchSpeed(-.5);
    climberWinchSubsystem.setRightWinchSpeed(-.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((
      Math.abs(climberWinchSubsystem.getLeftEncoder() - leftStart) >= 500000 || 
      Math.abs(climberWinchSubsystem.getRightEncoder() - rightStart) >= 500000) || 
      (Timer.getFPGATimestamp() - startTime) >= 1);
  }
}
