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

public class ClimberArmSubsystem extends SubsystemBase {
  /** Creates a new ClimberArmSubsystem. */
  private WPI_TalonSRX leftArm, rightArm;
  private MotorUtils leftMotorUtil, rightMotorUtil;
  private PowerDistribution m_PowerDistPanel;
  
  public ClimberArmSubsystem(PowerDistribution m_PowerDistPanel) {
    this.m_PowerDistPanel = m_PowerDistPanel;
    leftArm = new WPI_TalonSRX(Constants.CLIMBER_LEFT_ARM_ID);
    rightArm = new WPI_TalonSRX(Constants.CLIMBER_RIGHT_ARM_ID);

    leftArm.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    leftArm.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    rightArm.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    rightArm.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    
    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxEncoder("Left Arm Encoder", 100, leftArm));
    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxEncoder("Right Arm Encoder", 100, rightArm));
    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Left Arm Forward Switch", leftArm, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.FORWARD));
    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Left Arm Reverse Switch", leftArm, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.REVERSE));
    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Right Arm Forward Switch", rightArm, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.FORWARD));
    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Right Arm Reverse Switch", rightArm, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.REVERSE));
    
    leftArm.setNeutralMode(NeutralMode.Coast);
    rightArm.setNeutralMode(NeutralMode.Coast);
    
    leftArm.setInverted(true);
  }

  public void setSpeed(double speed) {
    leftArm.set(ControlMode.PercentOutput, speed);
    rightArm.set(ControlMode.PercentOutput, speed);
  }

  public void setLeftArmSpeed(double speed) {
    leftArm.set(ControlMode.PercentOutput, speed);
  }

  public void setRightArmSpeed(double speed) {
    rightArm.set(ControlMode.PercentOutput, speed);  
  }

  public void stopArms() {
    leftArm.set(ControlMode.PercentOutput, 0);
    rightArm.set(ControlMode.PercentOutput, 0);
  }
  public void stopLeftArm() {
    leftArm.set(ControlMode.PercentOutput, 0);
  }

  public void stopRightArm() {
    rightArm.set(ControlMode.PercentOutput, 0);
  }

  public double getLeftEncoder() {
    return leftArm.getSelectedSensorPosition();
  }

  public double getRightEncoder() {
    return rightArm.getSelectedSensorPosition();
  }

  public boolean isLeftStalled() {
    return leftMotorUtil.isStalled();
  }

  public boolean isRightStalled() {
    return rightMotorUtil.isStalled();
  }

  public double getLeftCurrent() {
    return m_PowerDistPanel.getCurrent(Constants.PDP_CLIMBER_L_ARM);
  }

  public double getRightCurrent() {
    return m_PowerDistPanel.getCurrent(Constants.PDP_CLIMBER_R_ARM);
  }

  public boolean getLeftHookSwitch() {
    return leftArm.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getLeftBotSensor() {
    return leftArm.getSensorCollection().isRevLimitSwitchClosed();
  }

  public boolean getRightHookSwitch() {
    return rightArm.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getRightBotSensor() {
    return rightArm.getSensorCollection().isRevLimitSwitchClosed();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.ENABLE_DEBUG) {
      SmartShuffleboard.put("Climber", "R Arm Encoder", getRightEncoder());
      SmartShuffleboard.put("Climber", "L Arm Encoder", getLeftEncoder());
      SmartShuffleboard.put("Climber", "R Arm Current", getRightCurrent());
      SmartShuffleboard.put("Climber", "L Arm Current", getLeftCurrent());
      SmartShuffleboard.put("Climber", "L Hook Switch", getLeftHookSwitch());
      SmartShuffleboard.put("Climber", "R Hook Switch", getRightHookSwitch());
      

      // Note this will change the stalled variable in isStalled
      SmartShuffleboard.put("Climber", "Left Arm Stalled", isLeftStalled());
      SmartShuffleboard.put("Climber", "Right Arm Stalled", isRightStalled());
    }
  }
}
