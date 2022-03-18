package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

public class WaitCommand extends LoggedCommandBase {
  private Timer timer = new Timer();
  private double duration;

  /**
   * Creates a new IntakeCommand.
   */
  public WaitCommand(double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    duration = seconds;
    addLog(seconds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute(){}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }
}

