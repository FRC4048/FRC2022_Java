package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class RUMBLE extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private XboxController xboxController;

  /**
   * Creates a new IntakeCommand.
   */
  public RUMBLE(XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xboxController = xboxController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xboxController.setRumble(GenericHID.RumbleType.kRightRumble, Constants.RUMBLE_STRENGTH);
    xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, Constants.RUMBLE_STRENGTH);
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
