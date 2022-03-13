// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoTargetSequence;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.drive.MoveDistance;
import frc.robot.commands.intakecommands.IntakeSequence;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.LimeLightVision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoShotSequence extends SequentialCommandGroup {
  /** Creates a new Autonomous. */
  public TwoShotSequence(TurretSubsystem TurretSubsystem, double TurretSpeed, Hood HoodSubsystem, double HoodSpeed, IntakeSubsystem IntakeSubsystem, Shooter Shooter, DriveTrain DriveTrain, double DriveTrainSpeed, double distanceInches, LimeLightVision Vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new SetShootingPosition(TurretSubsystem, TurretSpeed, HoodSubsystem, HoodSpeed),
      new ShootSequence(IntakeSubsystem, Shooter),
      new IntakeSequence(IntakeSubsystem),
      new MoveDistance(DriveTrain, DriveTrainSpeed, distanceInches),
      new AutoTargetSequence(TurretSubsystem, Vision,HoodSubsystem),
      new ShootSequence(IntakeSubsystem, Shooter)
    );
  }
}
