package frc.robot.commands;

import java.util.function.DoubleSupplier;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class Drive extends CommandBase {
  private final DriveTrain driveTrain;
  private final DoubleSupplier leftSpeed;
  private final DoubleSupplier rightSpeed;

  /**
   * Creates a new Drive.
   */
  public Drive(DriveTrain driveTrain, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    this.leftSpeed = leftSpeed; 
    this.rightSpeed = rightSpeed;
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      driveTrain.drive(-leftSpeed.getAsDouble(), -rightSpeed.getAsDouble(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}