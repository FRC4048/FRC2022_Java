// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.logging.LogCommandWrapper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TargetToSetLocation extends SequentialCommandGroup {
  /** Creates a new TargetManualSequence. */
  public TargetToSetLocation(Hood hood, TurretSubsystem turretSubsystem, double angle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LogCommandWrapper(new MoveHoodToAngle(hood, angle))
     // new LogCommandWrapper(new MoveTurretToAngle())
    );
  }
}
