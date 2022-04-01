// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber.ClimberArmSubsystem;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;
import frc.robot.utils.logging.LogCommandWrapper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractClimberSequence extends SequentialCommandGroup {
  /** Creates a new RetractClimberSequence. */
  public RetractClimberSequence(ClimberArmSubsystem climberArmSubsystem, ClimberWinchSubsystem climberWinchSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LogCommandWrapper(new MoveClimberParallelSequence(climberArmSubsystem, climberWinchSubsystem, -1)),
      new LogCommandWrapper(new ExtendClimberSolenoid(climberArmSubsystem))
    );
  }
}