// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.Shooter;

public class ExtendShooterPiston extends LoggedCommand {
  /** Creates a new RaisePiston. */
  private Shooter shooterSubsystem;
  public ExtendShooterPiston(Shooter shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void loggedInitialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void loggedExecute() {
    shooterSubsystem.extendPiston();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void loggedEnd(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean loggedIsFinished() {
    return true;
  }
}
