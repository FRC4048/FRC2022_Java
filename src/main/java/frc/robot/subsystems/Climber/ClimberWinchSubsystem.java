// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagTalonSrxEncoder;
import frc.robot.utils.diag.DiagTalonSrxSwitch;

public class ClimberWinchSubsystem extends SubsystemBase {
  /** Creates a new ClimberWinchSubsystem. */
  private WPI_TalonSRX leftWinch, rightWinch;
  public ClimberWinchSubsystem() {
    leftWinch = new WPI_TalonSRX(Constants.CLIMBER_LEFT_WINCH_ID);
    rightWinch = new WPI_TalonSRX(Constants.CLIMBER_RIGHT_WINCH_ID);

    leftWinch.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    rightWinch.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxEncoder("Left Winch Encoder", 100, leftWinch));
    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxEncoder("Right Winch Encoder", 100, rightWinch));
    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Left Winch Switch", leftWinch, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.REVERSE));
    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Right Winch Switch", rightWinch, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.FORWARD));
  
    leftWinch.setNeutralMode(NeutralMode.Brake);
    rightWinch.setNeutralMode(NeutralMode.Brake);

    leftWinch.setInverted(true);
    rightWinch.setInverted(true);
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
      SmartShuffleboard.put("Climber", "Right Arm Encoder", getRightEncoder());
      SmartShuffleboard.put("Climber", "Left Arm Encoder", getLeftEncoder());
      SmartShuffleboard.put("Climber", "Right Arm Voltage", getRightVoltage());
      SmartShuffleboard.put("Climber", "Left Arm Voltage", getLeftVoltage());
    }
  }
}
