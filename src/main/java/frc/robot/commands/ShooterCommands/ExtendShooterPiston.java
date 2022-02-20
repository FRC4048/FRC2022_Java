// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.Shooter;

import com.fasterxml.jackson.databind.jsontype.impl.SubTypeValidator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendShooterPiston extends CommandBase {
  /** Creates a new ExtendShooterPiston. */
  private Shooter shooterSubsytem;
  public ExtendShooterPiston(Shooter shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsytem = shooterSubsytem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsytem.extendPiston();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
