// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class HoodAutoCommand extends CommandBase {
  /** Creates a new HoodAuto. */
  private Hood hoodSubsystem;
  private DoubleSupplier verticalOffset;

  public HoodAutoCommand(Hood hoodSubsystem, DoubleSupplier verticalOffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hoodSubsystem = hoodSubsystem;
    this.verticalOffset = verticalOffset;
    addRequirements(hoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (verticalOffset.getAsDouble() >= 0) {
      hoodSubsystem.setHood(Constants.HOOD_AUTO_MOTOR_SPEED);
    }
    else {
      hoodSubsystem.setHood(-Constants.HOOD_AUTO_MOTOR_SPEED);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.stopHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(verticalOffset.getAsDouble()) < Constants.HOOD_AUTO_LIMIT;
  }
}
