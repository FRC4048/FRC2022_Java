// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.DriveCommands.MoveDistance;
import frc.robot.commands.DriveCommands.TurnDegrees;
import frc.robot.commands.ShooterCommands.AutoTargetSequence;
import frc.robot.commands.ShooterCommands.NonVisionParallelShoot;
import frc.robot.commands.ShooterCommands.ShooterSequeunce;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.LimeLightVision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoShotSequenceMiddle extends SequentialCommandGroup {
  /** Creates a new TwoShotSequenceRight. 
   * @param vision 
   * @param hoodSubsystem */
  public TwoShotSequenceMiddle(TurretSubsystem turretSubsystem, double turretSpeed, IntakeSubsystem intakeSubsystem, DriveTrain driveTrain, double speed, double distanceMeters, ShooterSubsystem shooterSubsystem, LimeLightVision limeLightVision, Hood hood) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //UNUSED RIGHT NOW
    addCommands(
      new MoveAndMoveHood(driveTrain, speed, 0.3, hood),
      new NonVisionParallelShoot(shooterSubsystem, intakeSubsystem, 11800),
      new ParallelMoveAndTurretReset(driveTrain, speed, 3.0, turretSubsystem, Constants.AUTO_TURRET_SPEED, intakeSubsystem),
      new TurnDegrees(driveTrain, -105),
      new ParallelMoveAndIntake(driveTrain, speed, 1.5, turretSubsystem, turretSpeed, intakeSubsystem, hood, turretSubsystem),
      new TurnDegrees(driveTrain, 30),
      new MoveDistance(driveTrain, speed, 1),
      new AutoTargetSequence(turretSubsystem, limeLightVision, hood),
      new ShooterSequeunce(shooterSubsystem, limeLightVision, turretSubsystem)
    );
  }
}
