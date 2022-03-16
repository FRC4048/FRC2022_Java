// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommandBase;

import frc.robot.subsystems.ShooterSubsystem;

public class StartShooterMotor extends CommandBase {
  /** Creates a new RotateShooterMotor. */
  private ShooterSubsystem shooterSubsystem;
  private double speed;
  private double initTime, timeout;

  public StartShooterMotor(ShooterSubsystem shooterSubsystem, double speed, double timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.speed = speed;
    this.timeout = timeout;

    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setShooterSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - initTime >= timeout) {
      return true;
  }
    return false;
  }
}
