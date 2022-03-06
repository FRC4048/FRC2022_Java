// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.utils.limelight.LimeLightVision;

public class HoodAutoCommand extends CommandBase {
  /** Creates a new HoodAuto. */
  private Hood hoodSubsystem;
  private double target;
  private LimeLightVision vision;

  public HoodAutoCommand(Hood hoodSubsystem, LimeLightVision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hoodSubsystem = hoodSubsystem;
    this.vision = vision;
    addRequirements(hoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = hoodSubsystem.calcPosition(vision.calcHorizontalDistanceToTarget(vision.getCameraAngles().getTx()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.getCameraAngles().getTx() >= 0) {
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
    return (hoodSubsystem.getEncoder() - target < 20) && (hoodSubsystem.getEncoder() - target > -20);
  }
}
