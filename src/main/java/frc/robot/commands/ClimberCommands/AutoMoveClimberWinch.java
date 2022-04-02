// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.commands.ClimberCommands.AutoMoveClimberArm.ClimberDirection;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;

public class AutoMoveClimberWinch extends LoggedCommandBase {
  /** Creates a new MoveClimberWinch. */
  ClimberWinchSubsystem climberWinchSubsystem;
  private double initTime;
  private ClimberDirection direction;

  public AutoMoveClimberWinch(ClimberWinchSubsystem climberWinchSubsystem, ClimberDirection direction) {
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
    if (direction == ClimberDirection.RETRACT) {
      climberWinchSubsystem.movePiston(false);
    }
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double directionMultiplier = 1;
    if (direction == ClimberDirection.RETRACT) {
      directionMultiplier = -1;
    }

    climberWinchSubsystem.setSpeed(Constants.CLIMBER_WINCH_SPEED * directionMultiplier);

    if (direction == ClimberDirection.RETRACT && climberWinchSubsystem.getRightOnBarSwitch() && climberWinchSubsystem.getLeftOnBarSwitch()) {
      climberWinchSubsystem.movePiston(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberWinchSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((Timer.getFPGATimestamp() - initTime) >= Constants.CLIMBER_ARM_TIMEOUT) {
      return true;
    }

    if (direction == ClimberDirection.RETRACT) {
      return ((climberWinchSubsystem.getLeftOnBarSwitch() && climberWinchSubsystem.getRightOnBarSwitch()) ||
          (climberWinchSubsystem.isLeftEverStalled() && climberWinchSubsystem.isRightEverStalled()));
    }

    return 
    (climberWinchSubsystem.getRightStrapExtendedSwitch() && climberWinchSubsystem.getLeftStrapExtendedSwitch() && (direction == ClimberDirection.EXTEND));
  }
}
