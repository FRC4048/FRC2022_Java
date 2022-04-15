// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.HoodCommands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.utils.limelight.CameraAngles;
import frc.robot.utils.limelight.LimeLightVision;
import frc.robot.utils.logging.LogCommandWrapper;

public class HoodAutoCommand extends LoggedCommandBase {
  /** Creates a new HoodAuto. */
  private static Map <Integer, Double> angleLookupMap;
  private Hood hoodSubsystem;
  private LimeLightVision vision;
  private double startTime;
  private boolean done;

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

  public HoodAutoCommand(Hood hoodSubsystem, LimeLightVision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hoodSubsystem = hoodSubsystem;
    this.vision = vision;
  }

  @Override
  public void initialize(){
    startTime = Timer.getFPGATimestamp();
    done = false;
  }

  @Override
  public void execute(){
    if(vision.hasTarget()) {
      CommandScheduler.getInstance().schedule(new LogCommandWrapper(new MoveHoodToAngle(hoodSubsystem, calculateAngle(vision))));
      done = true;
    }
  }

  @Override
  public boolean isFinished() {
    return (done || ((Timer.getFPGATimestamp() - startTime) >= Constants.HOOD_TARGET_TIMEOUT));
  }

  private Double calculateAngle(LimeLightVision vision) {
    double tempDistance = vision.calcHorizontalDistanceToTarget(vision.getCameraAngles().getTy()) / 12 + hoodSubsystem.getHoodAdj();
    addLog(tempDistance);
    int distance = (int)Math.round(tempDistance);
    return angleLookupMap.get(distance);
  }
}
