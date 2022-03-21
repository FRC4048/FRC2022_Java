// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ToggleBlockerPiston extends CommandBase {
  /** Creates a new ExtendBlockerPiston. */
  private ShooterSubsystem shooter;

  private boolean desiredDirection = false;

  public ToggleBlockerPiston(ShooterSubsystem shooter, boolean desiredDirection) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.desiredDirection = desiredDirection;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setBlockPiston(desiredDirection);
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
