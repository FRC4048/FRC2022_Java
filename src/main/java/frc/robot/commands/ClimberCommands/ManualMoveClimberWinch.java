// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;

public class ManualMoveClimberWinch extends CommandBase {
  /** Creates a new ManualMoveClimberWinch. */
  private ClimberWinchSubsystem climberWinchSubsystem;
  private XboxController climberController;
  private RobotContainer robotContainer;

  public ManualMoveClimberWinch(ClimberWinchSubsystem climberWinchSubsystem, XboxController climberController, RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberWinchSubsystem = climberWinchSubsystem;
    this.climberController = climberController;
    this.robotContainer = robotContainer;
    addRequirements(climberWinchSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (robotContainer.canClimb) {
    double rightSpeed = 0, leftSpeed = 0;
    // Right and Left Ys move backwards (up is -1, down is +1)
    double joySpeed = -climberController.getRightY();

    if (Math.abs(joySpeed) > Constants.CLIMBER_DEAD_ZONE) {
      rightSpeed = Constants.CLIMBER_WINCH_SPEED * joySpeed;
      leftSpeed = Constants.CLIMBER_WINCH_SPEED * joySpeed;
    } 


    climberWinchSubsystem.setRightWinchSpeed(rightSpeed);
    climberWinchSubsystem.setLeftWinchSpeed(leftSpeed);
  }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberWinchSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
