// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.TARGETING_STATE;

public class ToggleTargetState extends CommandBase {
  /** Creates a new ToggleTargetState. */
  private TurretSubsystem turret;

  public ToggleTargetState(TurretSubsystem turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if ((turret.getTargetState() == TARGETING_STATE.SWEEP) || (turret.getTargetState() == TARGETING_STATE.LOCK)) {
      turret.setTargetState(TARGETING_STATE.OFF);
    } else {
      turret.setTargetState(TARGETING_STATE.SWEEP);
    }
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
