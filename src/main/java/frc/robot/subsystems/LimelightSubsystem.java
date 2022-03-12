// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.limelight.CameraAngles;
import frc.robot.utils.limelight.LimeLightVision;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  
  LimeLightVision vision;

  public LimelightSubsystem() {
    vision = new LimeLightVision(Constants.CAMERA_HEIGHT, Constants.TARGET_HEIGHT, Constants.CAMERA_ANGLE);
    addToDashboard();
  }

  public void addToDashboard() {
    String cameraFeedUrl = "http://" + Constants.LIMELIGHT_IP_ADDR + ":5800/stream.mjpg";
    HttpCamera limelightFeed = new HttpCamera("limelight", cameraFeedUrl);
    ShuffleboardTab dashboardTab = Shuffleboard.getTab("Driver");
    dashboardTab.add("Limelight feed", limelightFeed).withProperties(Map.of("Show Crosshair", true, "Show Controls", false));

    SmartShuffleboard.put("Driver", "Limelight", "Feed URL", cameraFeedUrl);

  }

  public LimeLightVision getLimeLightVision() {
    return vision;
  }

  @Override
  public void periodic() {
    SmartShuffleboard.put("Driver", "Limelight", "Targeted", vision.hasTarget());
    CameraAngles angles = vision.getCameraAngles();
    if (angles != null) {
      SmartShuffleboard.put("Driver", "Limelight", "X Offset", angles.getTx());
      SmartShuffleboard.put("Driver", "Limelight", "Y Offset", angles.getTy());
      SmartShuffleboard.put("Driver", "Limelight", "Horiz Distance", vision.calcHorizontalDistanceToTarget(angles.getTy()));
    }
    else {
      SmartShuffleboard.put("Driver", "Limelight", "X Offset", 0);
      SmartShuffleboard.put("Driver", "Limelight", "Y Offset", 0);
      SmartShuffleboard.put("Driver", "Limelight", "Horiz Distance", 0);
    }
    // This method will be called once per scheduler run
  }
} 
 