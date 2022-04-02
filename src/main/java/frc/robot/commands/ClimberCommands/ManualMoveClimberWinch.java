// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;

public class ManualMoveClimberWinch extends CommandBase {
  /** Creates a new ManualMoveClimberWinch. */
  private ClimberWinchSubsystem climberWinchSubsystem;
  private XboxController climberController;

  public ManualMoveClimberWinch(ClimberWinchSubsystem climberWinchSubsystem, XboxController climberController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberWinchSubsystem = climberWinchSubsystem;
    this.climberController = climberController;
    addRequirements(climberWinchSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightSpeed = 0, leftSpeed = 0;
    // Right and Left Ys move backwards (up is -1, down is +1)
    double joySpeed = -climberController.getRightY();

    if (joySpeed > Constants.CLIMBER_DEAD_ZONE) {
      rightSpeed = Constants.CLIMBER_WINCH_SPEED * joySpeed;
      leftSpeed = Constants.CLIMBER_WINCH_SPEED * joySpeed;
    } 
    else if (joySpeed < -Constants.CLIMBER_DEAD_ZONE){      
        leftSpeed = joySpeed*Constants.CLIMBER_WINCH_SPEED;
        rightSpeed = joySpeed*Constants.CLIMBER_WINCH_SPEED;
    }

    if (climberController.getRightTriggerAxis() > 0.5) {
      rightSpeed *= Constants.CLIMBER_SLOW_WINCH_RATE; 
    } else if (climberController.getLeftTriggerAxis() > 0.5) {
      leftSpeed *= Constants.CLIMBER_SLOW_WINCH_RATE;
    }

    climberWinchSubsystem.setRightWinchSpeed(rightSpeed);
    climberWinchSubsystem.setLeftWinchSpeed(leftSpeed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberWinchSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
