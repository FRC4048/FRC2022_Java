package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitCommand extends LoggedCommand {
  private Timer timer = new Timer();
  private double duration;

  /**
   * Creates a new IntakeCommand.
   */
  public WaitCommand(double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    duration = seconds;

  }

  // Called when the command is initially scheduled.
  @Override
  public void loggedInitialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void loggedExecute(){}

  // Called once the command ends or is interrupted.
  @Override
  public void loggedEnd(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean loggedIsFinished() {
    return timer.hasElapsed(duration);
  }
}

