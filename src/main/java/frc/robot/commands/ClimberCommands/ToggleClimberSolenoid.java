// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.ClimberArmSubsystem;

public class ToggleClimberSolenoid extends CommandBase {
  /** Creates a new MoveClimberSolenoid. */
  private ClimberArmSubsystem climberArmSubsystem;
  private boolean state;
  public ToggleClimberSolenoid(ClimberArmSubsystem climberArmSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberArmSubsystem = climberArmSubsystem;
    this.state = state;
    addRequirements(climberArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberArmSubsystem.movePiston(!climberArmSubsystem.getPistonState());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
