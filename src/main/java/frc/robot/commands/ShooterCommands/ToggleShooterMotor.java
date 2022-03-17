// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ToggleShooterMotor extends CommandBase {
  /** Creates a new SpinShooter. */
  private ShooterSubsystem shooterSubsystem;
  private double startTime;
  public ToggleShooterMotor(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setRunning(!shooterSubsystem.isRunning());
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.isRunning()) {
      shooterSubsystem.setShooterRPM(Constants.SHOOTER_RPM);
    } else {
      shooterSubsystem.stopShooter();
    }
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return((Timer.getFPGATimestamp() - startTime) >= Constants.SHOOTER_TIMEOUT);
  }
}
