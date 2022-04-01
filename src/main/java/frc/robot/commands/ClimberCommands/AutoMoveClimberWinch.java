// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.commands.ClimberCommands.AutoMoveClimberArm.Direction;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;

public class AutoMoveClimberWinch extends LoggedCommandBase {
  /** Creates a new MoveClimberWinch. */
  ClimberWinchSubsystem climberWinchSubsystem;
  private double initTime;
  private Direction direction;
  private boolean lStall, rStall;


  public AutoMoveClimberWinch(ClimberWinchSubsystem climberWinchSubsystem, Direction direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberWinchSubsystem = climberWinchSubsystem;
    this.direction = direction;
    addRequirements(climberWinchSubsystem);
    addLog(direction.name());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
    lStall = false;
    rStall = false;
    //Reset Motor Utils Timeout
    climberWinchSubsystem.isLeftStalled();
    climberWinchSubsystem.isRightStalled();
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightSpeed = 0, leftSpeed = 0;
    if (direction == Direction.UP) {
      if (!climberWinchSubsystem.getLeftTopSwitch()) {
        leftSpeed = Constants.CLIMBER_WINCH_SPEED;
      }
      if (!climberWinchSubsystem.getRightTopSwitch()) {
        rightSpeed = Constants.CLIMBER_WINCH_SPEED;
      }
    } else {
      if (climberWinchSubsystem.isLeftStalled()) {
        lStall = true;
      }
      if (climberWinchSubsystem.isRightStalled()) {
        rStall = true;
      }
      if (!lStall) {
        leftSpeed = -Constants.CLIMBER_WINCH_SPEED;
      }
      if (!rStall) {
        rightSpeed = -Constants.CLIMBER_WINCH_SPEED;
      }    
    }

    climberWinchSubsystem.setLeftWinchSpeed(leftSpeed);
    climberWinchSubsystem.setRightWinchSpeed(rightSpeed);
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
    return ((rStall && lStall && (direction == Direction.DOWN)) || 
    (climberWinchSubsystem.getRightTopSwitch() && climberWinchSubsystem.getLeftTopSwitch() && (direction == Direction.UP)) ||
    ((Timer.getFPGATimestamp() - initTime) >= Constants.CLIMBER_ARM_TIMEOUT));
  }
}
