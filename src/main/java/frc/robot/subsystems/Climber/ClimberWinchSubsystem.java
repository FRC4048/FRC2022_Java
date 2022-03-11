// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberWinchSubsystem extends SubsystemBase {
  /** Creates a new ClimberWinchSubsystem. */
  private TalonSRX leftWinch, rightWinch;
  private DigitalInput leftTopSwitch, leftBotSwitch, rightTopSwitch, rightBotSwitch;
  public ClimberWinchSubsystem() {
    leftWinch = new TalonSRX(Constants.CLIMBER_LEFT_WINCH_ID);
    rightWinch = new TalonSRX(Constants.CLIMBER_RIGHT_WINCH_ID);

    leftWinch.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    rightWinch.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);



    leftWinch.setNeutralMode(NeutralMode.Brake);
    rightWinch.setNeutralMode(NeutralMode.Brake);

  }

  public void setSpeed(double speed) {
    leftWinch.set(ControlMode.PercentOutput, speed);
    rightWinch.set(ControlMode.PercentOutput, speed);
  }

  public void setLeftWinchSpeed(double speed) {
    leftWinch.set(ControlMode.PercentOutput, speed);
  }

  public void setRightWinchSpeed(double speed) {
    rightWinch.set(ControlMode.PercentOutput, speed);
  }

  public void stopLeftWinch() {
    leftWinch.set(ControlMode.PercentOutput, 0);
  }

  public void stopRightWinch() {
    rightWinch.set(ControlMode.PercentOutput, 0);
  }

  public double getLeftEncoder() {
    return leftWinch.getSelectedSensorPosition();
  }

  public double getRightEncoder() {
    return rightWinch.getSelectedSensorPosition();
  }

  public double getLeftVoltage() {
    return leftWinch.getBusVoltage();
  }

  public double getRightVolatage() {
    return rightWinch.getBusVoltage();
  }
  
  public double getLeftVelocity() {
    return leftWinch.getActiveTrajectoryVelocity();
  }

  public double getRightVelocity() {
    return rightWinch.getActiveTrajectoryVelocity();
  }

  public boolean getLeftSwitch() {
    return leftWinch.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getRightSwitch() {
    return rightWinch.getSensorCollection().isFwdLimitSwitchClosed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
