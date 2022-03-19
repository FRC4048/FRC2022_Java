// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands.MoveDistance;
import frc.robot.commands.IntakeCommand.IntakeSequence;
import frc.robot.commands.ShooterCommands.ShooterParallelSequeunce;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.LimeLightVision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoShotSequenceMiddle extends SequentialCommandGroup {
  /** Creates a new TwoShotSequenceMiddle. */
  public TwoShotSequenceMiddle(TurretSubsystem turretSubsystem, IntakeSubsystem intakeSubsystem, DriveTrain driveTrain, ShooterSubsystem shooterSubsystem, LimeLightVision limeLightVision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShooterParallelSequeunce(shooterSubsystem, intakeSubsystem, limeLightVision, turretSubsystem, null),
      new IntakeSequence(intakeSubsystem),
      new MoveDistance(driveTrain, Constants.AUTO_MOVE_SPEED, Constants.AUTO_DISTANCE_INCHES),
      new AutoSetShootingPosition(turretSubsystem, Constants.AUTO_TURRET_SPEED, Constants.AUTO_TURRET_CENTER_ANGLE),
      new ShooterParallelSequeunce(shooterSubsystem, intakeSubsystem, limeLightVision, turretSubsystem, null)
    );
  }
}