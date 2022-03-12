// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Climber.ClimberArmSubsystem;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;
import frc.robot.utils.logging.LogCommandWrapper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveClimberParallelSequence extends ParallelCommandGroup {
  /** Creates a new ExtendClimberParallelSequence. */
  private ClimberArmSubsystem climberArmSubsystem;
  private ClimberWinchSubsystem climberWinchSubsystem;
  private double length;
  private double speed;
  public MoveClimberParallelSequence(ClimberArmSubsystem climberArmSubsystem, ClimberWinchSubsystem climberWinchSubsystem, double speed, double length) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
     // new LogCommandWrapper(new MoveClimberArm(climberArmSubsystem, speed)),
      //new LogCommandWrapper(new MoveClimberWinch(climberWinchSubsystem, speed, length))
    );
  }
}
