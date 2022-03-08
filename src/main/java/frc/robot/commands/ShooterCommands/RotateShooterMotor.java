// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.LoggedCommand;

public class RotateShooterMotor extends LoggedCommand {
  /** Creates a new RotateShooterMotor. */
  private Shooter shooterSubsytem;
  private double speed;
  private double initTime;

  public RotateShooterMotor(Shooter shooterSubsytem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsytem = shooterSubsytem;
    this.speed = speed;

    addRequirements(shooterSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void loggedInitialize() {
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void loggedExecute() {
    shooterSubsytem.setShooterSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void loggedEnd(boolean interrupted) {
    shooterSubsytem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean loggedIsFinished() {
    if (Timer.getFPGATimestamp() - initTime >= Constants.SHOOTER_TIMEOUT) {
      return true;
  }
    return false;
  }
}
