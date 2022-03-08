// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Miscellaneous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
<<<<<<< HEAD
import frc.robot.commands.LoggedCommand;
import frc.robot.utils.limelight.LimeLightVision;
=======
>>>>>>> 64884a24969aa88fcc00b1a256fa615ef4a8cfa2

public class SetPipeline extends LoggedCommand {
  private int pipeline;
  
  /** Creates a new SetPipeline0. */
  public SetPipeline(int pipeline) {
    this.pipeline = pipeline;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void loggedInitialize() {
    addLog(pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void loggedExecute() {
    Robot.getRobotContainer().getLimeLight().setPipeline(pipeline);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void loggedEnd(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean loggedIsFinished() {
    return true;
  }
}