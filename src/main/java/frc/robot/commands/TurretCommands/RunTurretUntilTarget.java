// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.LimeLightVision;

public class RunTurretUntilTarget extends CommandBase {
  /** Creates a new TurretSweep. */
  private TurretSubsystem turretSubsystem;
  private boolean limeLightSight;
  private LimeLightVision limeLightVision;
  public RunTurretUntilTarget(TurretSubsystem turretSubsystem, LimeLightVision limeLightVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = turretSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.getRobotContainer().getLimeLight().setLedOn();
    Robot.getRobotContainer().getLimeLight().setPipeline(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limeLightSight = limeLightVision.hasTarget();
    turretSubsystem.setTurret(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turretSubsystem.setTurret(0);
    Robot.getRobotContainer().getLimeLight().setLedOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (limeLightSight == true) {
      return true;
    }
    else {
      return false;
    }
  }
}