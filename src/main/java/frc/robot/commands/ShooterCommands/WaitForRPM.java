// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.SmartShuffleboard;

public class WaitForRPM extends LoggedCommandBase {
  /** Creates a new WaitForRPM. */
  private double targetRPM;
  ShooterSubsystem shooter;

  public WaitForRPM(ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetRPM = shooter.getVelocity() * .29;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartShuffleboard.put("Shooter", "Wait Done?", shooter.getShooterRPM() >= targetRPM * .9);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartShuffleboard.put("Shooter", "Wait Done?", shooter.getShooterRPM() >= targetRPM * .9);
    return (shooter.getShooterRPM() >= targetRPM * .9);
  }
}
