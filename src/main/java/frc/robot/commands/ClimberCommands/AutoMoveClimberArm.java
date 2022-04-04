// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.subsystems.Climber.ClimberArmSubsystem;

public class AutoMoveClimberArm extends LoggedCommandBase {
  /** Creates a new ManualMoveClimberArm. */
  private ClimberArmSubsystem climberArmSubsystem;
  private double initTime;

  public enum ClimberDirection {
    EXTEND, RETRACT
  }

  private ClimberDirection direction;

  public AutoMoveClimberArm(ClimberArmSubsystem climberArmSubsystem, ClimberDirection direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberArmSubsystem = climberArmSubsystem;
    this.direction = direction;
    addRequirements(climberArmSubsystem);
    addLog(direction.name());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightSpeed = 0, leftSpeed = 0;
    // if (autoBalance) {
    // encoderDifference = Math.abs(climberArmSubsystem.getRightEncoder() -
    // climberArmSubsystem.getLeftEncoder());

    // if (climberArmSubsystem.getRightVelocity() < 1 ||
    // climberArmSubsystem.isRightStalled()) {
    // rightSpeed = 0;
    // }
    // if (climberArmSubsystem.getLeftVelocity() < 1 ||
    // climberArmSubsystem.isLeftStalled()) {
    // leftSpeed = 0;
    // }

    // if (Math.abs(climberArmSubsystem.getRightEncoder()) >
    // Math.abs(climberArmSubsystem.getLeftEncoder())+Constants.CLIMBER_MAX_ENCODER_DIFF)
    // {
    // rightSpeed *=
    // (1-(encoderDifference/Constants.CLIMBER_MAX_ENCODER_DIFF*Constants.CLIMBER_MIN_ARM_SPEED));
    // } else if (Math.abs(climberArmSubsystem.getLeftEncoder()) >
    // Math.abs(climberArmSubsystem.getRightEncoder())+Constants.CLIMBER_MAX_ENCODER_DIFF)
    // {
    // leftSpeed *=
    // (1-(encoderDifference/Constants.CLIMBER_MAX_ENCODER_DIFF*Constants.CLIMBER_MIN_ARM_SPEED));
    // }
    // }
    if (direction == ClimberDirection.EXTEND) {
      if (!climberArmSubsystem.getLeftBotSensor()) {
        leftSpeed = -Constants.CLIMBER_ARM_SPEED;
      }
      if (!climberArmSubsystem.getRightBotSensor()) {
        rightSpeed = -Constants.CLIMBER_ARM_SPEED;
      }
    } else { 
      if (!climberArmSubsystem.getLeftTopSensor()) {
        leftSpeed = Constants.CLIMBER_ARM_SPEED;
      }
      if (!climberArmSubsystem.getRightTopSensor()) {
        rightSpeed = Constants.CLIMBER_ARM_SPEED;
      }
    }

    climberArmSubsystem.setRightArmSpeed(rightSpeed);
    climberArmSubsystem.setLeftArmSpeed(leftSpeed);

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
    return 
        (((climberArmSubsystem.getRightBotSensor() && climberArmSubsystem.getLeftBotSensor() && (direction == ClimberDirection.EXTEND)) 
        ||
        (climberArmSubsystem.getRightTopSensor() && climberArmSubsystem.getLeftTopSensor() && (direction == ClimberDirection.RETRACT)) 
        ||
        ((Timer.getFPGATimestamp() - initTime) >= Constants.CLIMBER_ARM_TIMEOUT)));

  }
}
