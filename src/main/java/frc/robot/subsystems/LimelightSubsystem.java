// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.limelight.LimeLightVision;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  
  LimeLightVision vision;
  public LimelightSubsystem() {
    vision = new LimeLightVision(38, 104, 30);
  }

  @Override
  public void periodic() {
    if (vision.getCameraAngles() != null) {
      SmartShuffleboard.put("Limelight", "Y Offset", vision.getCameraAngles().getTy());
      SmartShuffleboard.put("Limelight", "Camera Distance", vision.calcHorizontalDistanceToTarget(vision.getCameraAngles().getTy()));
    }
    // This method will be called once per scheduler run
  }
} 
