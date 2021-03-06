// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.SetStateOff;
import frc.robot.commands.ToggleBlockerPiston;
import frc.robot.commands.IntakeCommand.DropBallCommand;
import frc.robot.commands.Miscellaneous.SetPipeline;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.logging.LogCommandWrapper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorSequence extends SequentialCommandGroup {
  /** Creates a new ElevatorSequence. */
  private ShooterSubsystem shooterSubsystem;
 
  public ElevatorSequence(ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LogCommandWrapper(new ToggleBlockerPiston(shooterSubsystem, true)),
      new LogCommandWrapper(new WaitForRPM(shooterSubsystem)),
      new LogCommandWrapper(new SetStateOff()),
      new LogCommandWrapper(new ExtendShooterPiston(shooterSubsystem)), 
      new LogCommandWrapper(new WaitCommand(Constants.SHOOTER_PISTON_WAIT)),
      new LogCommandWrapper(new RetractShooterPiston(shooterSubsystem)),
      new LogCommandWrapper(new SetShooterMotor(shooterSubsystem, 0)),
      new LogCommandWrapper(new ToggleBlockerPiston(shooterSubsystem, false))
    );
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      shooterSubsystem.setVelocity(0);
      shooterSubsystem.setBlockPiston(false);
      shooterSubsystem.retractPiston();
    }
  }
}
