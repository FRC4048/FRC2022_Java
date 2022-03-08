package frc.robot.commands.intakecommands;

<<<<<<< HEAD
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.IntakeSubsystem;
=======
>>>>>>> 64884a24969aa88fcc00b1a256fa615ef4a8cfa2
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

<<<<<<< HEAD
public class DropBallCommand extends LoggedCommand {
private IntakeSubsystem intakeSubsystem;
private double initTime;
=======
public class DropBallCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private double initTime;
>>>>>>> 64884a24969aa88fcc00b1a256fa615ef4a8cfa2

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DropBallCommand(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void loggedInitialize() {
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
<<<<<<< HEAD
  public void loggedExecute() {
      intakeSubsystem.spinMotor(Constants.INTAKE_MOTOR_SPEED);
=======
  public void execute() {
    intakeSubsystem.spinMotor(Constants.INTAKE_MOTOR_SPEED);
>>>>>>> 64884a24969aa88fcc00b1a256fa615ef4a8cfa2
  }

  // Called once the command ends or is interrupted.
  @Override
  public void loggedEnd(boolean interrupted) {
    intakeSubsystem.spinMotor(0);
  }

  // Returns true when the command should end.
  @Override
<<<<<<< HEAD
  public boolean loggedIsFinished() {
    if (!intakeSubsystem.getIntakeSensor() || (Timer.getFPGATimestamp() - initTime) >= Constants.RAISED_INTAKE_TIMEOUT) {
        return true;
=======
  public boolean isFinished() {
    if (!intakeSubsystem.isBallInIntake() || (Timer.getFPGATimestamp() - initTime) >= Constants.RAISED_INTAKE_TIMEOUT) {
      return true;
>>>>>>> 64884a24969aa88fcc00b1a256fa615ef4a8cfa2
    }
    return false;
  }
}
