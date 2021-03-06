// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands.MoveDistance;
import frc.robot.commands.TurretCommands.ResetTurretEncoder;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CrossTheLineSequence extends ParallelCommandGroup {
  /** Creates a new CrossTheLineSequence. */
  public CrossTheLineSequence(DriveTrain driveTrain, TurretSubsystem turretSubsystem, double turretSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new MoveDistance(driveTrain, 0.4, 1.8),
    new AutoSetShootingPosition(turretSubsystem, turretSpeed, Constants.AUTO_TURRET_CENTER_ANGLE)

    );
  }
}
