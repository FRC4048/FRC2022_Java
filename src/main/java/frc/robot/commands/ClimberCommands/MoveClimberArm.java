// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberArmSubsystem;

public class MoveClimberArm extends CommandBase {
  /** Creates a new MoveClimberArm. */
  ClimberArmSubsystem climberArmSubsystem;
  private double initTime;
  private double speed;
  
  public MoveClimberArm(ClimberArmSubsystem climberArmSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberArmSubsystem = climberArmSubsystem;
    addRequirements(climberArmSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberArmSubsystem.setRightArmSpeed(speed);
    climberArmSubsystem.setLeftArmSpeed(speed);
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climberArmSubsystem.getLeftArmVoltage() > 20) {
      climberArmSubsystem.stopLeftArm();
    }
    if (climberArmSubsystem.getRightArmVoltage() > 20) {
      climberArmSubsystem.stopLeftArm();
    }
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (climberArmSubsystem.getLeftArmVoltage() == 0 && climberArmSubsystem.getRightArmVoltage() == 0) || Timer.getFPGATimestamp() - initTime >= Constants.CLIMBER_TIMEOUT;
  }
}
