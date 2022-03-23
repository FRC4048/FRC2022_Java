// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;

public class WinchExtend extends CommandBase {
  /** Creates a new WinchExtend. */
  private ClimberWinchSubsystem climberWinchSubsystem;

  public WinchExtend(ClimberWinchSubsystem climberWinchSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberWinchSubsystem = climberWinchSubsystem;
    addRequirements(climberWinchSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberWinchSubsystem.setRightWinchSpeed(-.3);
    climberWinchSubsystem.setLeftWinchSpeed(-.3);

    if (!climberWinchSubsystem.getRightSwitch()) {
      climberWinchSubsystem.setRightWinchSpeed(0);
    }

    if (!climberWinchSubsystem.getLeftSwitch()) {
      climberWinchSubsystem.setLeftWinchSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberWinchSubsystem.setRightWinchSpeed(0);
    climberWinchSubsystem.setLeftWinchSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!climberWinchSubsystem.getLeftSwitch() && !climberWinchSubsystem.getRightSwitch());
  }
}
