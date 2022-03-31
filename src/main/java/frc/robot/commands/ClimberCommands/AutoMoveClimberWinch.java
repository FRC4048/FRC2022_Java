// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;

public class AutoMoveClimberWinch extends LoggedCommandBase {
  /** Creates a new MoveClimberWinch. */
  ClimberWinchSubsystem climberWinchSubsystem;
  private double initTime;
  private double direction;
  private double barContactTimeout;
  boolean autoBalance;


  public AutoMoveClimberWinch(ClimberWinchSubsystem climberWinchSubsystem, double direction, boolean autoBalance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberWinchSubsystem = climberWinchSubsystem;
    this.autoBalance = autoBalance;
    this.direction = direction;
    addRequirements(climberWinchSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
    climberWinchSubsystem.setSpeed(Constants.CLIMBER_WINCH_SPEED * direction);
    
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climberWinchSubsystem.isLeftStalled()) {
      climberWinchSubsystem.setLeftWinchSpeed(0);
    }

    if (climberWinchSubsystem.isRightStalled()) {
      climberWinchSubsystem.setRightWinchSpeed(0);
    }

    if (climberWinchSubsystem.isRightBarContact() != climberWinchSubsystem.isLeftBarContact()) {
      barContactTimeout = Timer.getFPGATimestamp();
    } else {
      barContactTimeout = 0;
    }
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
    return (climberWinchSubsystem.getRightVoltage() == 0 && climberWinchSubsystem.getLeftVoltage() == 0) || initTime > Constants.CLIMBER_ARM_TIMEOUT;
  }
}
