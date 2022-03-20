package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeWait extends LoggedCommandBase {
  private Timer timer = new Timer();
  private double duration;

  /**
   * Creates a new IntakeCommand.
   */
  public IntakeWait() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    addLog(duration);
    if (DriverStation.isAutonomous()) {    
        duration = 0.8;
    } else duration = 2;
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

