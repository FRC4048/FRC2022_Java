// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IMUSubsystem extends SubsystemBase {
  /** Creates a new IMUSubststem. */
  private ADIS16470_IMU gyro;

  public IMUSubsystem() {
    gyro = new ADIS16470_IMU();
  }

  // add @params Please do this before a competition or while the robot is
  // perfectly still
  public void resetGyro() {
    gyro.reset();
    gyro.calibrate();
  }

  public double getAccelY() {
    return gyro.getAccelY();
  }

  public double getAccelX() {
    return gyro.getAccelX();
  }

  public double getAccelZ() {
    return gyro.getAccelZ();
  }

  public double getAngle() {
    return gyro.getAngle();
  }

  public ADIS16470_IMU getGyro() {
    return gyro;
  }

  public void name() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
