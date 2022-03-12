// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberArmSubsystem;

public class AutoMoveClimberArm extends CommandBase {
  /** Creates a new ManualMoveClimberArm. */
  private ClimberArmSubsystem climberArmSubsystem;
  private double encoderDifference;
  private double direction;
  private double initTime;


  public AutoMoveClimberArm(ClimberArmSubsystem climberArmSubsystem, int direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberArmSubsystem = climberArmSubsystem;
    this.direction = direction;
    addRequirements(climberArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
    climberArmSubsystem.setSpeed(Constants.CLIMBER_ARM_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // WORK IN PROGRESS WILL BE DONE LATER
    encoderDifference = Math.abs(climberArmSubsystem.getRightEncoder() - climberArmSubsystem.getLeftEncoder());
    double rightSpeed = Constants.CLIMBER_ARM_SPEED * direction, leftSpeed = Constants.CLIMBER_ARM_SPEED * direction;

    if (climberArmSubsystem.getRightVelocity() < 1 || climberArmSubsystem.isRightStalled()) {
      rightSpeed = 0;
    }
    if (climberArmSubsystem.getLeftVelocity() < 1 || climberArmSubsystem.isLeftStalled()) {
      leftSpeed = 0;
    }

    if (Math.abs(climberArmSubsystem.getRightEncoder()) > Math.abs(climberArmSubsystem.getLeftEncoder())+Constants.CLIMBER_MAX_ENCODER_DIFF) {
      rightSpeed *= (1-(encoderDifference/Constants.CLIMBER_MAX_ENCODER_DIFF*Constants.CLIMBER_MIN_ARM_SPEED));
    } else if (Math.abs(climberArmSubsystem.getLeftEncoder()) > Math.abs(climberArmSubsystem.getRightEncoder())+Constants.CLIMBER_MAX_ENCODER_DIFF) {
      leftSpeed *= (1-(encoderDifference/Constants.CLIMBER_MAX_ENCODER_DIFF*Constants.CLIMBER_MIN_ARM_SPEED));
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
    return (climberArmSubsystem.getRightVoltage() == 0 && climberArmSubsystem.getLeftVoltage() == 0) || Timer.getFPGATimestamp() - initTime >= Constants.CLIMBER_ARM_TIMEOUT;
  }
}
