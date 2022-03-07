package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class RaiseIntakeCommand extends LoggedCommand {
  private IntakeSubsystem intakeSubsystem;

  /**
   * Creates a new IntakeCommand.
   */
  public RaiseIntakeCommand(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void loggedInitialize() {
    intakeSubsystem.retractPiston();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void loggedExecute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void loggedEnd(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean loggedIsFinished() {
    return true;
  }
}

