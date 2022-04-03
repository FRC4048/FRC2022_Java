// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Climber.ClimberArmSubsystem;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;
import frc.robot.utils.limelight.LimeLightVision;
import frc.robot.utils.logging.LogCommandWrapper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InitialClimbSequence extends SequentialCommandGroup {
  /** Creates a new InitialClimbSequence. */
  public InitialClimbSequence(ClimberArmSubsystem climberArmSubsystem, ClimberWinchSubsystem climberWinchSubsystem, TurretSubsystem turretSubsystem, LimeLightVision vision, XboxController climberController) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LogCommandWrapper(new InitialExtendSequeunce(climberArmSubsystem, climberWinchSubsystem, turretSubsystem, vision)),
      new LogCommandWrapper(new RetractWinchForTimeout(climberWinchSubsystem)),
      new LogCommandWrapper(new ConfirmTransition(climberController)),
      new LogCommandWrapper(new RetractClimberSequence(climberWinchSubsystem, climberArmSubsystem))
    );
  }
}
