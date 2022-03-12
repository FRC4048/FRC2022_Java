package frc.robot.commands.IntakeCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.logging.LogCommandWrapper;

public class IntakeSequence extends SequentialCommandGroup {

  /**
   * Creates a new IntakeCommand.
   */
  IntakeSubsystem intakeSubsystem;
  public IntakeSequence(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;

    addCommands(
        new LogCommandWrapper(new DeployIntakeCommand(intakeSubsystem)),
        new LogCommandWrapper(new IntakeBallCommand(intakeSubsystem)),
        new LogCommandWrapper(new RaiseIntakeCommand(intakeSubsystem)),
        new LogCommandWrapper(new WaitCommand(2)),
        new LogCommandWrapper(new DropBallCommand(intakeSubsystem))
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
    new LogCommandWrapper(new RaiseIntakeCommand(intakeSubsystem));
    super.end(interrupted);
  }
}

