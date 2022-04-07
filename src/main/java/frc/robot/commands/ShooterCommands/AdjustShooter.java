// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.ShooterSubsystem;

public class AdjustShooter extends CommandBase {
  /** Creates a new AdjustShooter. */
  private ShooterSubsystem shooter;
  private double adjustment;
  private Hood hood;

  public AdjustShooter(ShooterSubsystem shooter, Hood hood, double adjustment) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.adjustment = adjustment;
    this.hood = hood;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterAdj(adjustment);
    hood.setHoodAdj(adjustment);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
