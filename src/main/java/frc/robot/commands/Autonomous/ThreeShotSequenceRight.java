// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.DriveCommands.AutoTurnDegrees;
import frc.robot.commands.DriveCommands.MoveDistance;
import frc.robot.commands.HoodCommands.MoveHoodToAngle;
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
public class ThreeShotSequenceRight extends SequentialCommandGroup {
  /** Creates a new TwoShotSequenceRight. 
   * @param vision 
   * @param hoodSubsystem */
  public ThreeShotSequenceRight(TurretSubsystem turretSubsystem, double turretSpeed, IntakeSubsystem intakeSubsystem, DriveTrain driveTrain, double speed, double distanceInches, ShooterSubsystem shooterSubsystem, LimeLightVision limeLightVision, Hood hood) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new MoveHoodToAngle(hood, 109.0),
      //new NonVisionParallelShoot(shooterSubsystem, intakeSubsystem, 12000.0),
      new ParallelMoveAndTurretResetAndIntake(driveTrain, 0.4, 40, turretSubsystem, turretSpeed, intakeSubsystem, hood),
      new AutoTargetSequence(turretSubsystem, limeLightVision, hood),
      new ShooterSequeunce(shooterSubsystem, limeLightVision),
      new WaitCommand(0.5),
      new ShooterSequeunce(shooterSubsystem, limeLightVision),
      new AutoTurnDegrees(driveTrain, -111),
      new ParralelMoveAndIntakeAndSetTurret(driveTrain, 0.5, 90, turretSubsystem, -turretSpeed, intakeSubsystem, hood, turretSubsystem),
      new AutoTurnDegrees(driveTrain, 65),
      new AutoTargetSequence(turretSubsystem, limeLightVision, hood),
      new ShooterSequeunce(shooterSubsystem, limeLightVision)
    );
  }
}
