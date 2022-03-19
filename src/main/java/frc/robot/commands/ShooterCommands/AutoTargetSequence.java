// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoTargetParallel;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.Miscellaneous.SetLEDOn;
import frc.robot.commands.Miscellaneous.SetPipeline;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.limelight.LimeLightVision;
import frc.robot.utils.logging.LogCommandWrapper;

public class AutoTargetSequence extends SequentialCommandGroup {

  /** Creates a new AutoTargetSequence. */
  private TurretSubsystem turretSubsystem;
  public AutoTargetSequence(TurretSubsystem turretSubsystem, LimeLightVision vision, Hood hoodSubsystem) {
    this.turretSubsystem = turretSubsystem;
//    vision = new LimeLightVision(38, 104, 30);

    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
            new LogCommandWrapper(new SetPipeline(Constants.LIMELIGHT_TARGET_DETECTION)),
            new LogCommandWrapper(new WaitCommand(0.1)),
            new LogCommandWrapper(new AutoTargetParallel(turretSubsystem, vision, hoodSubsystem))
    );
    SmartShuffleboard.put("Driver", "Data", "Can Shoot", true);
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
