// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;

public class WinchExtend extends CommandBase {
  /** Creates a new WinchExtend. */
  private ClimberWinchSubsystem climberWinchSubsystem;
  private double startTime;

  public WinchExtend(ClimberWinchSubsystem climberWinchSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberWinchSubsystem = climberWinchSubsystem;
    addRequirements(climberWinchSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double lSpeed = 0.5, rSpeed = 0.5;
    //SmartShuffleboard.put("Climber", "R Winch Cmd Sensor", climberWinchSubsystem.getRightSwitch());
    if (!climberWinchSubsystem.getRightSwitch()) {
      rSpeed = 0;
    }

    if (!climberWinchSubsystem.getLeftSwitch()) {
      lSpeed = 0;
    }

    climberWinchSubsystem.setRightWinchSpeed(rSpeed);
    climberWinchSubsystem.setLeftWinchSpeed(lSpeed);
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
    return ((!climberWinchSubsystem.getLeftSwitch() && !climberWinchSubsystem.getRightSwitch()) || 
            (Timer.getFPGATimestamp() - startTime >= Constants.EXTEND_WINCH_TIMEOUT));
  }
}
