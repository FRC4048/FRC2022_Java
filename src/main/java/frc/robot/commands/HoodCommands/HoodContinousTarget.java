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
  private static Map<Integer, Double> angleLookupMap;
  private Hood hood;
  private DoubleSupplier rightJoystickY;
  private LimeLightVision vision;
  private Double ticks;
  private static boolean hoodState;

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

  public HoodContinousTarget(Hood hood, DoubleSupplier rightJoystickY, LimeLightVision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hood = hood;
    this.rightJoystickY = rightJoystickY;
    this.vision = vision;

    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hoodState = false;
    switch (Robot.getTargetState()) {
      case OFF:
        if (Math.abs(rightJoystickY.getAsDouble()) < Constants.HOOD_JOYSTICK_THRESHOLD) {
          hood.setHood(0);
        } else {
          hood.setHood(rightJoystickY.getAsDouble() * Constants.HOOD_MOTOR_SPEED);
        }
        break;

      case LOCK:
        if (vision.hasTarget()) {
          ticks = calculateAngle(vision);
          if (ticks != null) {
            if (Math.abs(hood.getPotentiometer() - ticks) <= Constants.HOOD_ERROR_THRESHOLD) {
              hood.setHood(0);
            } else {
              double direction = Math.signum(hood.getPotentiometer() - ticks);
              hood.setHood(Constants.HOOD_AUTO_MOTOR_SPEED * direction);
            }
          }
        } else {
          hood.setHood(0);
        }
        break;
    }
    hood.setHoodLockState(hoodState);
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
    double distance = vision.calcHorizontalDistanceToTarget(vision.getCameraAngles().getTy()) / 12;
    if ((-.0858 * Math.pow(distance, 2) + 5.36 * distance + 79.7) < 106) {
      hoodState = false;
      return 106.0;
    } else if ((-.0858 * Math.pow(distance, 2) + 5.36 * distance + 79.7) > 145) {
      hoodState = false;
      return 145.0;
    } else {
      hoodState = true;
      return -.0858 * Math.pow(distance, 2) + 5.36 * distance + 79.7;
    }
  }
}
