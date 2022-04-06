// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.MotorUtils;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagSwitch;
import frc.robot.utils.diag.DiagTalonSrxEncoder;

public class ClimberWinchSubsystem extends SubsystemBase {
  /** Creates a new ClimberWinchSubsystem. */
  private WPI_TalonSRX leftWinch, rightWinch;
  private MotorUtils leftMotorStall, rightMotorStall;
  private DigitalInput leftSensor, rightSensor;
  private PowerDistribution powerDistribution;
  private Solenoid climberLPiston, climberRPiston;

  public ClimberWinchSubsystem(PowerDistribution m_PowerDistPanel) {
  
    leftWinch = new WPI_TalonSRX(Constants.CLIMBER_LEFT_WINCH_ID);
    rightWinch = new WPI_TalonSRX(Constants.CLIMBER_RIGHT_WINCH_ID);
    leftSensor = new DigitalInput(Constants.CLIMBER_L_WINCH_SENSOR);
    rightSensor = new DigitalInput(Constants.CLIMBER_R_WINCH_SENSOR);
    
    climberLPiston = new Solenoid(Constants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Constants.CLIMBER_L_PISTON_ID);

    powerDistribution = m_PowerDistPanel;

    leftMotorStall = new MotorUtils(Constants.PDP_CLIMBER_L_WINCH, Constants.WINCH_CURR_LIMIT, Constants.CLIMBER_WINCH_CURR_TIMEOUT, m_PowerDistPanel);
    rightMotorStall = new MotorUtils(Constants.PDP_CLIMBER_R_WINCH, Constants.WINCH_CURR_LIMIT, Constants.CLIMBER_WINCH_CURR_TIMEOUT, m_PowerDistPanel);

    leftWinch.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    rightWinch.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    //leftWinch.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    //rightWinch.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);


    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxEncoder("Left Winch Encoder", 100, leftWinch));
    Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxEncoder("Right Winch Encoder", 100, rightWinch));
    // Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Left Winch Switch", leftWinch, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.REVERSE));
    // Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Right Winch Switch", rightWinch, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.FORWARD));
    Robot.getDiagnostics().addDiagnosable(new DiagSwitch("L Winch Switch", leftSensor));
    Robot.getDiagnostics().addDiagnosable(new DiagSwitch("R Winch Switch", rightSensor));

    leftWinch.setNeutralMode(NeutralMode.Brake);
    rightWinch.setNeutralMode(NeutralMode.Brake);

    leftWinch.setInverted(true);
    rightWinch.setInverted(false);
  }

  public void setSpeed(double speed) {
    setLeftWinchSpeed(speed);
    setRightWinchSpeed(speed);
  }

  /**
   * Negative Speed is Retracting
   * Positive Speed is Expanding
   */
  public void setLeftWinchSpeed(double speed) {

    if (speed < 0) {
      // Retracting, chance of stall here
      if (leftMotorStall.everStalled()) {
        // Don't let it turn motor anymore
        speed = 0;
      }
    } else if (speed >= 0) {
      // Cant stall on expand, reset motor stall state
      leftMotorStall.resetStall();
    }

    leftWinch.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Negative Speed is Retracting
   * Positive Speed is Expanding
   */
  public void setRightWinchSpeed(double speed) {

    if (speed < 0) {
      // Retracting, chance of stall here
      if (rightMotorStall.everStalled()) {
        // Don't let it turn motor anymore
        speed = 0;
      }
    } else if (speed >= 0) {
      // Cant stall on expand, reset motor stall state
      rightMotorStall.resetStall();
    }

    rightWinch.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    stopLeftWinch();
    stopRightWinch();
  }

  public void movePiston(boolean state) {
    climberLPiston.set(state);
    climberRPiston.set(state);
  }

  public boolean getPistonState() {
    return climberLPiston.get();
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

  public boolean isLeftStalled() {
    return leftMotorStall.isStalled();
  }

  public boolean isLeftEverStalled() {
    return leftMotorStall.everStalled();
  }

  public boolean isRightStalled() {
    return rightMotorStall.isStalled();
  }

  public boolean isRightEverStalled() {
    return rightMotorStall.everStalled();
  }

  public double getLeftCurrent() {
    return powerDistribution.getCurrent(Constants.PDP_CLIMBER_L_WINCH);
  }

  public double getRightCurrent() {
    return powerDistribution.getCurrent(Constants.PDP_CLIMBER_R_WINCH);
  }

  /**
   * True when tripped, false when open
   */
  public boolean getLeftStrapExtendedSwitch() {
    return leftWinch.getSensorCollection().isFwdLimitSwitchClosed();
  }

  /**
   * True when tripped, false when open
   */
  public boolean getRightStrapExtendedSwitch() {
    return rightWinch.getSensorCollection().isFwdLimitSwitchClosed();
  }

  /** 
   * True when tripped, false when open
   */
  public boolean getLeftOnBarSwitch() {
    return !leftSensor.get();
  }

  /**
   * True when tripped, false when open
   */
  public boolean getRightOnBarSwitch() {
    return !rightSensor.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    isLeftStalled();
    isRightStalled();
    if (Constants.ENABLE_DEBUG) {
      SmartShuffleboard.put("Climber", "R Winch Encoder", getRightEncoder());
      SmartShuffleboard.put("Climber", "L Winch Encoder", getLeftEncoder());
      SmartShuffleboard.put("Climber", "R Winch Current", getRightCurrent());
      SmartShuffleboard.put("Climber", "L Winch Current", getLeftCurrent());
      SmartShuffleboard.put("Climber", "R Winch Strap Switch", getRightStrapExtendedSwitch());
      SmartShuffleboard.put("Climber", "L Winch Strap Switch", getLeftStrapExtendedSwitch());
      // THIS WAS INADVERTANTLY TRIGGERING THE STALL CHECK
      SmartShuffleboard.put("Climber", "L Winch Stalled", isLeftStalled());
      SmartShuffleboard.put("Climber", "R Winch Stalled", isRightStalled());
      SmartShuffleboard.put("Climber", "L Winch Ever Stalled", leftMotorStall.everStalled());
      SmartShuffleboard.put("Climber", "R Winch Ever Stalled", rightMotorStall.everStalled());
    }
      SmartShuffleboard.put("Climber", "R Winch Strap Switch", getRightStrapExtendedSwitch());
      SmartShuffleboard.put("Climber", "L Winch Strap Switch", getLeftStrapExtendedSwitch());
  }
} 
