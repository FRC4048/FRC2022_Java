// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.limelight.LimeLightVision;
import frc.robot.utils.logging.LogCommandWrapper;

public class VisionAutoShooter extends LoggedCommandBase {
  /** Creates a new AutoShooterToggle. */
  private static Map <Integer, Double> rpmLookupMap;
  private LimeLightVision vision;
  private ShooterSubsystem shooter;
  private double startTime;
  private boolean done;

  static {
    // Conversion Map from feet to pot ticks
    // Placeholder
    rpmLookupMap = new HashMap<>();
    rpmLookupMap.put(5, 11500.0);
    rpmLookupMap.put(6, 11800.0);
    rpmLookupMap.put(7, 12000.0);
    rpmLookupMap.put(8, 12100.0);
    rpmLookupMap.put(9, 12400.0);
    rpmLookupMap.put(10, 12600.0);
    rpmLookupMap.put(11, 12900.0);
    rpmLookupMap.put(12, 12900.0);
    rpmLookupMap.put(13, 13500.0);
    rpmLookupMap.put(14, 13500.0);
    rpmLookupMap.put(15, 13500.0);
    rpmLookupMap.put(16, 14200.0);
  }

  public VisionAutoShooter(LimeLightVision vision, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.shooter = shooter;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(vision.hasTarget()) {
      Double rpm = calculateRPM(vision);
      if (rpm != null) {
        CommandScheduler.getInstance().schedule(new LogCommandWrapper(new SetShooterMotor(shooter, rpm)));
        SmartShuffleboard.put("Shooter", "desiredSpeed", rpm);
      }
      else {
        addLog("UNKOWN DISTANCE - using default speed");
        CommandScheduler.getInstance().schedule(new LogCommandWrapper(new SetShooterMotor(shooter, Constants.SHOOTER_RPM)));
      }
      done = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (done || ((Timer.getFPGATimestamp() - startTime) >= Constants.HOOD_TARGET_TIMEOUT));
  }

  private Double calculateRPM(LimeLightVision vision) {
    double distance = vision.calcHorizontalDistanceToTarget(vision.getCameraAngles().getTy()) / 12;
    addLog(distance);
    SmartShuffleboard.put("Shooter", "Distance", distance);
    if (10565 * Math.pow(Math.E, .0177 * distance) > 15000) {
      return 15000.0;
    }
    return 10565 * Math.pow(Math.E, .0177 * distance);
  }
}
