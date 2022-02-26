package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class DropBallCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private double initTime;

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
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.spinMotor(Constants.INTAKE_MOTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.spinMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!intakeSubsystem.isBallInIntake() || (Timer.getFPGATimestamp() - initTime) >= Constants.RAISED_INTAKE_TIMEOUT) {
      return true;
    }
    return false;
  }
}
