// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class WaitAndToggleBlockerPiston extends SequentialCommandGroup {
  /** Creates a new ExtendBlockerPiston. */
  private ShooterSubsystem shooter;

  private boolean desiredDirection = false;

  public WaitAndToggleBlockerPiston(ShooterSubsystem shooter, boolean desiredDirection, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new WaitCommand(time),
      new ToggleBlockerPiston(shooter, desiredDirection)
    );
  }
}
