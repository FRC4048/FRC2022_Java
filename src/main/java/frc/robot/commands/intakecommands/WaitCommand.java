package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants2022Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class WaitCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private double initTime;
  /**
   * Creates a new IntakeCommand.
   */
  public WaitCommand(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - initTime >= Constants2022Robot.INTAKE_BUFFER) {
        return true;
    }
    return false;
  }
}

