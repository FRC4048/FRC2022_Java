// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MotorUtils;

public class ClimberArmSubsystem extends SubsystemBase {
  /** Creates a new ClimberArmSubsystem. */
  private TalonSRX leftArm, rightArm;
  private Solenoid climberPiston;
  private MotorUtils leftMotorUtil, rightMotorUtil;
  
  public ClimberArmSubsystem(PowerDistribution m_PowerDistPanel) {
    leftArm = new TalonSRX(Constants.CLIMBER_LEFT_MOTOR_ID);
    rightArm = new TalonSRX(Constants.CLIMBER_RIGHT_MOTOR_ID);
    climberPiston = new Solenoid(Constants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Constants.CLIMBER_PISTON_ID);

    leftMotorUtil = new MotorUtils(Constants.PDP_CLIMBER_L_ARM, Constants.CLIMBER_V_LIMIT, Constants.CLIMBER_ARM_V_TIMEOUT, m_PowerDistPanel);
    rightMotorUtil = new MotorUtils(Constants.PDP_CLIMBER_R_ARM, Constants.CLIMBER_V_LIMIT, Constants.CLIMBER_ARM_V_TIMEOUT, m_PowerDistPanel);

    leftArm.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    leftArm.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    rightArm.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    rightArm.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    
    leftArm.setNeutralMode(NeutralMode.Brake);
    leftArm.setNeutralMode(NeutralMode.Brake);
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

  public void movePiston(boolean state) {
    climberPiston.set(state);
  }

  public boolean getPistonState() {
    return climberPiston.get();
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

  public double getLeftVoltage() {
    return leftArm.getBusVoltage();
  }

  public double getRightVolatage() {
    return rightArm.getBusVoltage();
  }

  public boolean getLeftTopSensor() {
    return leftArm.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getLeftBotSensor() {
    return leftArm.getSensorCollection().isRevLimitSwitchClosed();
  }

  public boolean getRightTopSensor() {
    return rightArm.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getRightBotSensor() {
    return leftArm.getSensorCollection().isRevLimitSwitchClosed();
  }



  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
