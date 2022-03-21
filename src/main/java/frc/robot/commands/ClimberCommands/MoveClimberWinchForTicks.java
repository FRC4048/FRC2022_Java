// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberArmSubsystem;
import frc.robot.subsystems.Climber.ClimberWinchSubsystem;
import frc.robot.utils.MotorUtils;

public class MoveClimberWinchForTicks extends CommandBase {
  /** Creates a new MoveClimberArm. */
  private ClimberWinchSubsystem climberWinchSubsystem;
  private double initTime;
  private double speed;
  private int ticks;
  
  public MoveClimberWinchForTicks(ClimberWinchSubsystem climberWinchSubsystem, double speed, int ticks) {
    // Use addRequirements() here to declare subsystem dependencies.

    //WORK IN PROGRESS
    this.climberWinchSubsystem = climberWinchSubsystem;
    this.ticks = ticks;
    addRequirements(climberWinchSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberWinchSubsystem.setRightWinchSpeed(speed);
    climberWinchSubsystem.setLeftWinchSpeed(speed);
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climberWinchSubsystem.getRightEncoder() > ticks) {
      climberWinchSubsystem.setRightWinchSpeed(0);
    }
    if (climberWinchSubsystem.getLeftEncoder() > ticks) {
      climberWinchSubsystem.setLeftWinchSpeed(0);
    }
    
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return (climberWinchSubsystem.getLeftVoltage() == 0 && climberWinchSubsystem.getRightVoltage() == 0) || initTime > Constants.CLIMBER_ARM_TIMEOUT;
  }
}
