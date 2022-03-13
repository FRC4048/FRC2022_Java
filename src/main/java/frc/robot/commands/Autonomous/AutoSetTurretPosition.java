// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

/*Use after using the CalibrateTurretEncoderSequence */
public class AutoSetTurretPosition extends CommandBase {
  /** Creates a new AutoSetTurretPosition. */
  private TurretSubsystem TurretSubsystem;
  private double speed;
  private double Angle;
  public AutoSetTurretPosition(TurretSubsystem TurretSubsystem, double speed, double Angle) {
    this.TurretSubsystem = TurretSubsystem;
    this.speed = speed;
    this.Angle = Angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TurretSubsystem.resetEncoder();
    TurretSubsystem.setTurret(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    TurretSubsystem.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (TurretSubsystem.getEncoder() == Angle) {
      return true;
    }
    else {
      return false;
    }
  }
}
