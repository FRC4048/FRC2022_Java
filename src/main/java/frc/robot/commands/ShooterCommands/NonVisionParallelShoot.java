// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand.DeployIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.limelight.LimeLightVision;
import frc.robot.utils.logging.LogCommandWrapper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NonVisionParallelShoot extends ParallelCommandGroup {
  /** Creates a new ShootSequence. */
  public NonVisionParallelShoot(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, double rpm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LogCommandWrapper(new SetShooterMotor(shooterSubsystem, rpm)),
      new LogCommandWrapper(new ElevatorSequence(shooterSubsystem))
    );
  }
}
