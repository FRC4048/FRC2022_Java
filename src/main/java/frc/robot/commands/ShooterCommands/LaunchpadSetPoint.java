// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.HoodCommands.MoveHoodToAngle;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.logging.LogCommandWrapper;

public class LaunchpadSetPoint extends SequentialCommandGroup {
  /** Creates a new TarmacSetPoint. */
  public LaunchpadSetPoint(Hood hood, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new LogCommandWrapper(new MoveHoodToAngle(hood, 140.0)),
      new LogCommandWrapper(new SetShooterMotor(shooterSubsystem, 14000.0)),
      new LogCommandWrapper(new ElevatorSequence(shooterSubsystem)),
      new LogCommandWrapper(new WaitCommand(Constants.EXTRA_SHOOTER_SPIN_TIME)),
      new LogCommandWrapper(new SetShooterMotor(shooterSubsystem, 0))
    );
  }
}
