package frc.robot.commands.IntakeCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeBallCommand extends LoggedCommandBase {
  private IntakeSubsystem intakeSubsystem;
  private double initTime;
  private int ballDetections;
  //we don't want a timeout if we don't call it
  private int timeOut = 60;
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
  public IntakeBallCommand(IntakeSubsystem intakeSubsystem, int timeOut) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    this.timeOut = timeOut;
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
    if (intakeSubsystem.isBallInIntake()) {
      ballDetections++;
    } else {ballDetections = 0;}
    SmartDashboard.putNumber("Ball Detections", ballDetections);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.spinMotor(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((ballDetections >= 5) || (Timer.getFPGATimestamp() - initTime) >= timeOut);
  }
}
