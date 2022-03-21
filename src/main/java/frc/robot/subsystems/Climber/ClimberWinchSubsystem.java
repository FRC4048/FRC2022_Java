// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.MotorUtils;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagTalonSrxEncoder;
import frc.robot.utils.diag.DiagTalonSrxSwitch;

public class ClimberWinchSubsystem extends SubsystemBase {
  /** Creates a new ClimberWinchSubsystem. */
  private WPI_TalonSRX leftWinch, rightWinch;
  private MotorUtils leftMotorContact, rightMotorContact, leftMotorStall, rightMotorStall;
  public ClimberWinchSubsystem(PowerDistribution m_PowerDistPanel) {
    leftWinch = new WPI_TalonSRX(Constants.CLIMBER_LEFT_WINCH_ID);
    rightWinch = new WPI_TalonSRX(Constants.CLIMBER_RIGHT_WINCH_ID);

    leftMotorContact = new MotorUtils(Constants.PDP_CLIMBER_L_ARM, Constants.WINCH_CONTACT_V, Constants.WINCH_CONTACT_V_TIMEOUT, m_PowerDistPanel);
    rightMotorContact = new MotorUtils(Constants.PDP_CLIMBER_R_ARM, Constants.WINCH_CONTACT_V, Constants.WINCH_CONTACT_V_TIMEOUT, m_PowerDistPanel);
    leftMotorStall = new MotorUtils(Constants.PDP_CLIMBER_L_ARM, Constants.WINCH_V_LIMIT, Constants.CLIMBER_WINCH_V_TIMEOUT, m_PowerDistPanel);
    rightMotorStall = new MotorUtils(Constants.PDP_CLIMBER_R_ARM, Constants.WINCH_V_LIMIT, Constants.CLIMBER_WINCH_V_TIMEOUT, m_PowerDistPanel);

    leftWinch.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    rightWinch.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxEncoder("Left Winch Encoder", 100, leftWinch));
    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxEncoder("Right Winch Encoder", 100, rightWinch));
    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Left Winch Forward Switch", leftWinch, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.FORWARD));
    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Right Winch Forward Switch", rightWinch, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.REVERSE));
  
    leftWinch.setNeutralMode(NeutralMode.Brake);
    rightWinch.setNeutralMode(NeutralMode.Brake);

    leftWinch.setInverted(true);
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

  public boolean isLeftBarContact() {
    return leftMotorContact.isStalled();
  }

  public boolean isRightBarContact() {
    return rightMotorContact.isStalled();
  }

  public boolean isLeftStalled() {
    return leftMotorStall.isStalled();
  }

  public boolean isRightStalled() {
    return rightMotorStall.isStalled();
  }

  public double getLeftVoltage() {
    return leftWinch.getBusVoltage();
  }

  public double getRightVoltage() {
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
    if (Constants.ENABLE_DEBUG) {
      SmartShuffleboard.put("Climber", "Right Winch Voltage", getRightVoltage());
      SmartShuffleboard.put("Climber", "Left Winch Voltage", getLeftVoltage());
      SmartShuffleboard.put("Climber", "Right Winch Encoders", getRightEncoder());
      SmartShuffleboard.put("Climber", "Left Winch Encoders", getLeftEncoder());
    }
  }
}
