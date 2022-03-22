// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.\
package frc.robot.commands.TurretCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class RunTurretUntilLimitSwitch extends CommandBase {
  
  private TurretSubsystem turretSubsystem;

  private double startTime;

  public RunTurretUntilLimitSwitch(TurretSubsystem turretSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
    this.turretSubsystem = turretSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretSubsystem.setTurret(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stopTurret();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (turretSubsystem.getLeftSwitch() || (Timer.getFPGATimestamp() - startTime) >= Constants.TURRETSPIN_TIMEOUT);
  }
}
