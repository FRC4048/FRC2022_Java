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
    encoderDifference = Math.abs(climberArmSubsystem.getRightEncoder() - climberArmSubsystem.getLeftEncoder());

    if (climberArmSubsystem.getRightVolatage() != 0 || climberArmSubsystem.isRightStalled()) {
      if (Math.abs(climberArmSubsystem.getRightEncoder()) > Math.abs(climberArmSubsystem.getLeftEncoder())+Constants.CLIMBER_MAX_ENCODER_DIFF) {
        climberArmSubsystem.setRightArmSpeed(Constants.CLIMBER_ARM_SPEED * (1-(encoderDifference/Constants.CLIMBER_MAX_ENCODER_DIFF*Constants.CLIMBER_MIN_ARM_SPEED)));
      }
    }
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
    return (climberArmSubsystem.getRightVolatage() == 0 && climberArmSubsystem.getLeftVoltage() == 0) || Timer.getFPGATimestamp() - initTime >= Constants.CLIMBER_ARM_TIMEOUT;
  }
}
