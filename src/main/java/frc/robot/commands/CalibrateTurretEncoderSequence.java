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

public class CalibrateTurretEncoderSequence extends SequentialCommandGroup {

  /** Creates a new AutoTargetSequence. */
  private TurretSubsystem turretSubsystem;
  public CalibrateTurretEncoderSequence(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new RunTurretUntilLimitSwitch(turretSubsystem),
      new WaitCommand(1),
      new ResetTurretEncoder(turretSubsystem)
    );
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
