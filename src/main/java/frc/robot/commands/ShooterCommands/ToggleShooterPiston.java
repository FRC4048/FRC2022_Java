// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ToggleShooterPiston extends CommandBase {
  /** Creates a new TogglePiston. */
  private Shooter shooterSubsytem;

  public ToggleShooterPiston(Shooter shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsytem = shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override

  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooterSubsytem.getPistonState() == true) {
      shooterSubsytem.retractPiston();
    } else {
      shooterSubsytem.extendPiston();
    }
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
