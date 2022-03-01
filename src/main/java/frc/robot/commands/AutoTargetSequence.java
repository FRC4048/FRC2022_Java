// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Miscellaneous.SetLEDOn;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.limelight.LimeLightVision;

public class AutoTargetSequence extends SequentialCommandGroup {

  /** Creates a new AutoTargetSequence. */
  private TurretSubsystem turretSubsystem;
  public AutoTargetSequence(TurretSubsystem turretSubsystem, LimeLightVision vision, Hood hoodSubsystem) {
    this.turretSubsystem = turretSubsystem;
    vision = new LimeLightVision(38, 104, 30);

    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new SetLEDOn(),
      new TurretAuto(turretSubsystem, vision),
      new HoodAutoCommand(hoodSubsystem, vision)
    );
    SmartShuffleboard.put("Shooter", "Data", "Can Shoot", true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }
}
