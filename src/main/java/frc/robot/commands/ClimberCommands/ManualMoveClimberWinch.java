// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;

public class ManualMoveClimberWinch extends CommandBase {
  /** Creates a new ManualMoveClimberWinch. */
  private ClimberWinchSubsystem climberWinchSubsystem;
  private XboxController xboxController;
  public ManualMoveClimberWinch(ClimberWinchSubsystem climberWinchSubsystem, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberWinchSubsystem = climberWinchSubsystem;
    this.xboxController = xboxController;
    addRequirements(climberWinchSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (xboxController.getRightY() > 0.75) {
      climberWinchSubsystem.setSpeed(Constants.CLIMBER_WINCH_SPEED);
    } else if (xboxController.getRightY() < -0.75) {
      climberWinchSubsystem.setSpeed(-Constants.CLIMBER_WINCH_SPEED);
    } else {
      climberWinchSubsystem.setSpeed(0);
    }

  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
