package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSequence extends SequentialCommandGroup {
  private IntakeSubsystem intakeSubsystem;

  /**
   * Creates a new IntakeCommand.
   */
  public IntakeSequence(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);

    addCommands(
        new DeployIntakeCommand(intakeSubsystem),
        new IntakeBallCommand(intakeSubsystem),
        new RaiseIntakeCommand(intakeSubsystem),
        new DropBallCommand(intakeSubsystem)
    );
  }

  //Overrides left in in case future additions are necessary
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      super.end(interrupted);
  }
}
