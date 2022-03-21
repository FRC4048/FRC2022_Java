// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberArmSubsystem;
import frc.robot.utils.MotorUtils;

public class MoveClimberArmForTicks extends CommandBase {
  /** Creates a new MoveClimberArm. */
  private ClimberArmSubsystem climberArmSubsystem;
  private double initTime;
  private double speed;
  private int ticks;
  
  public MoveClimberArmForTicks(ClimberArmSubsystem climberArmSubsystem, double speed, int ticks) {
    // Use addRequirements() here to declare subsystem dependencies.

    //WORK IN PROGRESS
    this.climberArmSubsystem = climberArmSubsystem;
    this.ticks = ticks;
    addRequirements(climberArmSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberArmSubsystem.setRightArmSpeed(speed);
    climberArmSubsystem.setLeftArmSpeed(speed);
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climberArmSubsystem.getRightEncoder() > ticks) {
      climberArmSubsystem.setRightArmSpeed(0);
    }
    if (climberArmSubsystem.getLeftEncoder() > ticks) {
      climberArmSubsystem.setLeftArmSpeed(0);
    }
    
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return (climberArmSubsystem.getLeftVoltage() == 0 && climberArmSubsystem.getRightVoltage() == 0) || initTime > Constants.CLIMBER_ARM_TIMEOUT;
  }
}
