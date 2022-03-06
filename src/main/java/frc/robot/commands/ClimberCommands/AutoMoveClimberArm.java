// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberArmSubsystem;

public class AutoMoveClimberArm extends CommandBase {
  /** Creates a new ManualMoveClimberArm. */
  private ClimberArmSubsystem climberArmSubsystem;
  private XboxController xboxController;
  private double encoderDifference;

  public AutoMoveClimberArm(ClimberArmSubsystem climberArmSubsystem, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberArmSubsystem = climberArmSubsystem;
    this.xboxController = xboxController;
    addRequirements(climberArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
/*
    encoderDifference = Math.abs(climberArmSubsystem.getLeftEncoder()-climberArmSubsystem.getRightEncoder());

    if (climberArmSubsystem.getLeftArmVoltage() < 20) {
      if (Math.abs(climberArmSubsystem.getLeftEncoder()) > Math.abs(climberArmSubsystem.getRightEncoder())) {
        if (xboxController.getLeftY() > 0.5) {
          climberArmSubsystem.setLeftArmSpeed((Constants.CLIMBER_ARM_SPEED * (1-(encoderDifference/Constants.CLIMBER_MAX_ENCODER_DIFF*Constants.CLIMBER_MIN_ARM_SPEED))));
        } else if (xboxController.getLeftY() < 0.5) {
          climberArmSubsystem.setLeftArmSpeed(-1 * (Constants.CLIMBER_ARM_SPEED * (1-(encoderDifference/Constants.CLIMBER_MAX_ENCODER_DIFF*Constants.CLIMBER_MIN_ARM_SPEED))));
        } else {
          climberArmSubsystem.setLeftArmSpeed(0);
        }
      } else {
        climberArmSubsystem.setLeftArmSpeed(Constants.CLIMBER_ARM_SPEED);
      }  
    } else {
      climberArmSubsystem.setLeftArmSpeed(0);
    }

    if (climberArmSubsystem.getRightArmVoltage() < 20) {
      if (Math.abs(climberArmSubsystem.getRightEncoder()) > Math.abs(climberArmSubsystem.getLeftEncoder())) {
        if (xboxController.getLeftY() > 0.5) {
          climberArmSubsystem.setRightArmSpeed((Constants.CLIMBER_ARM_SPEED * (1-(encoderDifference/Constants.CLIMBER_MAX_ENCODER_DIFF*Constants.CLIMBER_MIN_ARM_SPEED))));
        } else if (xboxController.getLeftY() < 0.5) {
          climberArmSubsystem.setRightArmSpeed(-1 * (Constants.CLIMBER_ARM_SPEED * (1-(encoderDifference/Constants.CLIMBER_MAX_ENCODER_DIFF*Constants.CLIMBER_MIN_ARM_SPEED))));
        } else {
          climberArmSubsystem.setRightArmSpeed(0);
        }
      } else {
        climberArmSubsystem.setRightArmSpeed(Constants.CLIMBER_ARM_SPEED);
      }     
    } else {
      climberArmSubsystem.setRightArmSpeed(0);
    }
    */
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
    return false;
  }
}
