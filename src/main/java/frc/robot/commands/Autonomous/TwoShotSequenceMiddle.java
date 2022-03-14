// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.MoveDistance;
import frc.robot.commands.IntakeCommand.IntakeSequence;
import frc.robot.commands.ShooterCommands.ShooterParallelSequeunce;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoShotSequenceMiddle extends SequentialCommandGroup {
  /** Creates a new TwoShotSequenceMiddle. */
  public TwoShotSequenceMiddle(TurretSubsystem TurretSubsystem, double TurretSpeed, double Angle, IntakeSubsystem IntakeSubsystem, DriveTrain DriveTrain, double Speed, double DistanceInches, ShooterSubsystem ShooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoSetShootingPosition(TurretSubsystem, TurretSpeed, Angle),
      new ShooterParallelSequeunce(ShooterSubsystem, IntakeSubsystem),
      new IntakeSequence(IntakeSubsystem),
      new MoveDistance(DriveTrain, Speed, DistanceInches),
      new AutoSetShootingPosition(TurretSubsystem, TurretSpeed, Angle),
      new ShooterParallelSequeunce(ShooterSubsystem, IntakeSubsystem)
    );
  }
}
