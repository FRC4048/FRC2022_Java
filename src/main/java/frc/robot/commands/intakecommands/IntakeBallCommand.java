package frc.robot.commands.intakecommands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeBallCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
private IntakeSubsystem intakeSubsystem;
private double initTime;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeBallCommand(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      intakeSubsystem.spinMotor(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.spinMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (intakeSubsystem.getIntakeSensor() || (Timer.getFPGATimestamp() - initTime) > 5/*change this to Constants.INTAKE_TIMEOUT after constants file added*/) {
        return true;
    }
    return false;
  }
}
