// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.HoodCommands;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Hood;
import frc.robot.utils.limelight.LimeLightVision;

public class HoodContinousTarget extends CommandBase {
  /** Creates a new HoodContinousTarget. */
  private static Map <Integer, Double> angleLookupMap;
  private Hood hood;
  private DoubleSupplier rightJoystickY;
  private LimeLightVision vision;

  static {
    // Conversion Map from feet to pot ticks
    // Placeholder
    angleLookupMap = new HashMap<>();
    angleLookupMap.put(5, 106.0);
    angleLookupMap.put(6, 106.0);
    angleLookupMap.put(7, 112.0);
    angleLookupMap.put(8, 121.0);
    angleLookupMap.put(9, 121.0);
    angleLookupMap.put(10, 121.0);
    angleLookupMap.put(11, 129.0);
    angleLookupMap.put(12, 131.5);
    angleLookupMap.put(13, 136.0);
    angleLookupMap.put(14, 139.0);
    angleLookupMap.put(15, 140.3);
    angleLookupMap.put(16, 142.9);
  }

  public HoodContinousTarget (Hood hood, DoubleSupplier rightJoystickY, LimeLightVision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hood = hood;
    this.rightJoystickY = rightJoystickY;
    this.vision = vision;

    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(Robot.getTargetState()) {
      case OFF:
        if(Math.abs(rightJoystickY.getAsDouble()) < Constants.HOOD_JOYSTICK_THRESHOLD){
          hood.setHood(0);
        }
        hood.setHood(rightJoystickY.getAsDouble()*Constants.HOOD_MOTOR_SPEED);
        break;

      case LOCK:
        if(vision.hasTarget()) {
          if (calculateAngle(vision) != null) {
            double direction = Math.signum(hood.getPotentiometer() - calculateAngle(vision));
            hood.setHood(Constants.HOOD_AUTO_MOTOR_SPEED * direction);
          }
        }
        break; 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.setHood(0);
    Robot.setTargetState(frc.robot.Robot.TARGETING_STATE.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private static Double calculateAngle(LimeLightVision vision) {
    double tempDistance = vision.calcHorizontalDistanceToTarget(vision.getCameraAngles().getTy()) / 12;
    int distance = (int)Math.round(tempDistance);
    return angleLookupMap.get(distance);
  }
}
