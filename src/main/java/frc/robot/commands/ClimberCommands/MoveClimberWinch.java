// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;

public class MoveClimberWinch extends CommandBase {
  /** Creates a new MoveClimberWinch. */
  ClimberWinchSubsystem climberWinchSubsystem;
  private double initTime;
  private double ticksToTurn;
  private double leftEncoderValue;
  private double rightEncoderValue;
  private double speed;

  public MoveClimberWinch(ClimberWinchSubsystem climberWinchSubsystem, double speed, double length) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberWinchSubsystem = climberWinchSubsystem;
    this.ticksToTurn = length * Constants.CLIMBER_TICKS_PER_INCH;
    addRequirements(climberWinchSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftEncoderValue = climberWinchSubsystem.getLeftEncoder();
    rightEncoderValue = climberWinchSubsystem.getRightEncoder();
    climberWinchSubsystem.setRightWinchSpeed(speed);
    climberWinchSubsystem.setLeftWinchSpeed(speed);
    initTime = Timer.getFPGATimestamp();
    
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climberWinchSubsystem.getLeftWinchVoltage() > 20 || Math.abs(climberWinchSubsystem.getLeftEncoder() - leftEncoderValue) > Math.abs(ticksToTurn)) {
      climberWinchSubsystem.stopLeftWinch();
    }
    if (climberWinchSubsystem.getRightWinchVoltage() > 20 || Math.abs(climberWinchSubsystem.getRightEncoder() - rightEncoderValue) > Math.abs(ticksToTurn)) {
      climberWinchSubsystem.stopRightWinch();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (climberWinchSubsystem.getLeftWinchVoltage() == 0 && climberWinchSubsystem.getRightWinchVoltage() == 0) || Timer.getFPGATimestamp() - initTime >= Constants.CLIMBER_TIMEOUT;
  }
}
