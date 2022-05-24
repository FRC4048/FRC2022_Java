// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.subsystems.DriveTrain;

/** An example command that uses an example subsystem. */
public class MoveDistance extends LoggedCommandBase {
  private final DriveTrain driveTrain;
  private double encoder;
  private double speed;
  private double distanceInches;
  private double startTime;
  
  
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
    addLog(distanceInches);
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.drive(speed, speed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("finished");
    driveTrain.drive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((distanceInches / 44.905922263461282401981639271768 < (driveTrain.getLeftEncoder() - encoder))) {
      return true;
    }
    else {
      return ((Timer.getFPGATimestamp() - startTime) >= Constants.MOVEDISTANCE_TIMEOUT);
    }
  }
}
