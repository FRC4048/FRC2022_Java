// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ManuallyToggleIntake extends LoggedCommandBase {
private IntakeSubsystem intakeSubsystem;
  /** Creates a new ToggleIntake. */
  public ManuallyToggleIntake(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem=intakeSubsystem;
    addRequirements(intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean pistonState = intakeSubsystem.getPiston1State();
    intakeSubsystem.togglePiston();
    if (pistonState) {
      intakeSubsystem.spinMotor(0);
    }
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
