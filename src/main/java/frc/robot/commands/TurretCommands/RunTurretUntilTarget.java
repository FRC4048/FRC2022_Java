// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.LimeLightVision;

public class RunTurretUntilTarget extends CommandBase {
  /** Creates a new TurretSweep. */
  private TurretSubsystem turretSubsystem;
  private LimeLightVision limeLightVision;
  private double turretSpeed;
  private double initTime;
  public RunTurretUntilTarget(TurretSubsystem turretSubsystem, LimeLightVision limeLightVision, double turretSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = turretSubsystem;
    this.limeLightVision = limeLightVision;
    this.turretSpeed = turretSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (turretSubsystem.getRightSwitch() && turretSpeed < 0) {
      turretSpeed = -turretSpeed;
    }
    if (turretSubsystem.getLeftSwitch() && turretSpeed > 0) {
      turretSpeed = -turretSpeed;
    }
    turretSubsystem.setTurret(turretSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turretSubsystem.setTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (limeLightVision.hasTarget() == true || Timer.getFPGATimestamp() - initTime >= Constants.TURRETSPIN_TIMEOUT) {
      return true;
    }
    else {
      return false;
    }
  }
}