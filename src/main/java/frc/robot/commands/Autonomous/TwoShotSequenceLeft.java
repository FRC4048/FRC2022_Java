// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.MoveDistance;
import frc.robot.commands.HoodCommands.MoveHoodToAngle;
import frc.robot.commands.ShooterCommands.AutoTargetSequence;
import frc.robot.commands.ShooterCommands.NonVisionParallelShoot;
import frc.robot.commands.ShooterCommands.NonVisionParallelShootDeployIntake;
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
public class TwoShotSequenceLeft extends SequentialCommandGroup {
  /** Creates a new Autonomous. 
  */
  public TwoShotSequenceLeft(TurretSubsystem turretSubsystem, double turretSpeed, IntakeSubsystem intakeSubsystem, DriveTrain driveTrain, double speed, double distanceMeters, ShooterSubsystem shooterSubsystem, LimeLightVision limeLightVision, Hood hood) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveAndMoveHood(driveTrain, 0.4, 0.3, hood),
      //new MoveHoodToAngle(hood, 106.0),
      new NonVisionParallelShootDeployIntake(shooterSubsystem, intakeSubsystem, 12000),
      new ParallelMoveAndTurretResetAndIntake(driveTrain, speed, 1.5, turretSubsystem, turretSpeed, intakeSubsystem, hood),
      new AutoTargetSequence(turretSubsystem, limeLightVision, hood),
      //new WaitCommand(0.8),
      new ShooterSequeunce(shooterSubsystem, limeLightVision, turretSubsystem)
    );
  }
}
