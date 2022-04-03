// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands.MoveDistance;
import frc.robot.commands.HoodCommands.MoveHoodToAngle;
import frc.robot.commands.IntakeCommand.IntakeSequence;
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
public class OneShotSequenceMiddle extends SequentialCommandGroup {
  /** Creates a new TwoShotSequenceMiddle. 
   * @param hood 
   * @param speed 
   * @param distanceInches */
  public OneShotSequenceMiddle(TurretSubsystem turretSubsystem, IntakeSubsystem intakeSubsystem, DriveTrain driveTrain, ShooterSubsystem shooterSubsystem, LimeLightVision limeLightVision, Hood hood, double speed, double distanceInches) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveAndMoveHood(driveTrain, speed, 12, hood),
      new NonVisionParallelShoot(shooterSubsystem, intakeSubsystem, 11900),
      new ParralelMoveAndTurretReset(driveTrain, speed, 65, turretSubsystem, Constants.AUTO_TURRET_SPEED, intakeSubsystem)
    );
  }
}
