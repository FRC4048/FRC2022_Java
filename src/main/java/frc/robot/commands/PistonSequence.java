// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Miscellaneous.SetLEDOff;
import frc.robot.commands.ShooterCommands.ExtendShooterPiston;
import frc.robot.commands.ShooterCommands.RetractShooterPiston;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.SmartShuffleboard;

public class PistonSequence extends SequentialCommandGroup {
  private Shooter shooter;
  /** Creates a new PistonSequence. */
  public PistonSequence() {
    addCommands(
      new WaitCommand(Constants.SHOOTER_SPINUP_DELAY),
      new ExtendShooterPiston(shooter),
new WaitCommand(Constants.PISTON_DELAY),
      new RetractShooterPiston(shooter),
      new SetLEDOff()
    );
    SmartShuffleboard.put("Shooter", "Data", "Can Shoot", false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }
}
