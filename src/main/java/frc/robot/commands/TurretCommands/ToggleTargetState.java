// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.Robot.TARGETING_STATE;
import frc.robot.commands.Miscellaneous.SetPipeline;
import frc.robot.utils.logging.LogCommandWrapper;

public class ToggleTargetState extends CommandBase {
  /** Creates a new ToggleTargetState. */

  public ToggleTargetState() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Robot.getTargetState() == TARGETING_STATE.LOCK) {
      Robot.setTargetState(TARGETING_STATE.OFF);
      CommandScheduler.getInstance().schedule(new LogCommandWrapper(new SetPipeline(0)));
    } else {
      Robot.setTargetState(TARGETING_STATE.LOCK);
      CommandScheduler.getInstance().schedule(new LogCommandWrapper(new SetPipeline(1)));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Robot.setTargetState(TARGETING_STATE.OFF);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
