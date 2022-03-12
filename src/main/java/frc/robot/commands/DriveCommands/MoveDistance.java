// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.Drive;

/** An example command that uses an example subsystem. */
public class MoveDistance extends CommandBase {
  private final DriveTrain driveTrain;
  private double encoder;
  private double speed;
  private double distanceInches;
  //Still need to assign this a value 1.0 is temporary
  private double encoderPerInch=1.0;

  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveDistance(DriveTrain driveTrain, double speed, double distanceInches) {
    this.distanceInches = distanceInches;
    this.speed = speed;
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoder = driveTrain.getLeftEncoder();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.drive(speed, speed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((encoderPerInch * distanceInches) < (driveTrain.getLeftEncoder() - encoder)) {
      return true;
    }
    else {
      return false;
    }
  }
}
