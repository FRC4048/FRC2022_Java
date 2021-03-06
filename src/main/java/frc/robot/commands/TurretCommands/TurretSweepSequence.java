// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Miscellaneous.SetPipeline;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.LimeLightVision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretSweepSequence extends SequentialCommandGroup {
  /** Creates a new TurretSweepSequence. */
  public TurretSweepSequence(TurretSubsystem turretSubsystem, LimeLightVision limeLightVision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetPipeline(Constants.LIMELIGHT_TARGET_DETECTION),
      new RunTurretUntilLimitSwitch(turretSubsystem),
      new RunTurretUntilTarget(turretSubsystem, limeLightVision, Constants.TURRET_SWEEP_SPEED)
    );
  }
}
