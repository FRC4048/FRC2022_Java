// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;

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

public class ClimberWinchSubsystem extends SubsystemBase {
  /** Creates a new ClimberWinchSubsystem. */
  private CANSparkMax leftWinch, rightWinch;
  private MotorUtils leftMotorStall, rightMotorStall;
  private DigitalInput leftSensor, rightSensor;
  private PowerDistribution powerDistribution;
  private Solenoid climberLPiston, climberRPiston;

  private SparkMaxLimitSwitch leftStrapSwitch;
  private SparkMaxLimitSwitch rightStrapSwitch;

  public ClimberWinchSubsystem(PowerDistribution m_PowerDistPanel) {
  
    leftWinch = new CANSparkMax(Constants.CLIMBER_LEFT_WINCH_ID, MotorType.kBrushless);
    rightWinch = new CANSparkMax(Constants.CLIMBER_RIGHT_WINCH_ID, MotorType.kBrushless);
    leftSensor = new DigitalInput(Constants.CLIMBER_L_WINCH_SENSOR);
    rightSensor = new DigitalInput(Constants.CLIMBER_R_WINCH_SENSOR);

    leftStrapSwitch = leftWinch.getForwardLimitSwitch(Type.kNormallyOpen);
    leftStrapSwitch.enableLimitSwitch(true);
    rightStrapSwitch = rightWinch.getForwardLimitSwitch(Type.kNormallyOpen);
    rightStrapSwitch.enableLimitSwitch(true);
    
    climberLPiston = new Solenoid(Constants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Constants.CLIMBER_L_PISTON_ID);

    powerDistribution = m_PowerDistPanel;

    leftMotorStall = new MotorUtils(Constants.PDP_CLIMBER_L_WINCH, Constants.WINCH_CURR_LIMIT, Constants.CLIMBER_WINCH_CURR_TIMEOUT, m_PowerDistPanel);
    rightMotorStall = new MotorUtils(Constants.PDP_CLIMBER_R_WINCH, Constants.WINCH_CURR_LIMIT, Constants.CLIMBER_WINCH_CURR_TIMEOUT, m_PowerDistPanel);

    Robot.getDiagnostics().addDiagnosable(new DiagSwitch("L Winch Switch", leftSensor));
    Robot.getDiagnostics().addDiagnosable(new DiagSwitch("R Winch Switch", rightSensor));

    leftWinch.restoreFactoryDefaults();
    rightWinch.restoreFactoryDefaults();

    leftWinch.setInverted(true);
    rightWinch.setInverted(false);

    leftWinch.setIdleMode(IdleMode.kBrake);
    rightWinch.setIdleMode(IdleMode.kBrake);
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
    } else if (speed > 0) {
      // Cant stall on expand, reset motor stall state
      leftMotorStall.resetStall();
    }

    leftWinch.set(speed);
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
    } else if (speed > 0) {
      // Cant stall on expand, reset motor stall state
      rightMotorStall.resetStall();
    }

    rightWinch.set(speed);
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
    leftWinch.set(0);
  }

  public void stopRightWinch() {
    rightWinch.set(0);
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
    return leftStrapSwitch.isPressed();
  }

  /**
   * True when tripped, false when open
   */
  public boolean getRightStrapExtendedSwitch() {
    return rightStrapSwitch.isPressed();
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

  public boolean getLeftSwitch() {
    return leftSensor.get();
  }

  public boolean getRightSwitch() {
    return rightSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    isLeftStalled();
    isRightStalled();
    if (Constants.ENABLE_DEBUG) {
      SmartShuffleboard.put("Climber", "R Winch Current", getRightCurrent());
      SmartShuffleboard.put("Climber", "L Winch Current", getLeftCurrent());
      SmartShuffleboard.put("Climber", "L Winch Sensor", getLeftSwitch());
      SmartShuffleboard.put("Climber", "R Winch Sensor", getRightSwitch());
      SmartShuffleboard.put("Climber", "R Winch Strap Switch", getRightStrapExtendedSwitch());
      SmartShuffleboard.put("Climber", "L Winch Strap Switch", getLeftStrapExtendedSwitch());
      SmartShuffleboard.put("Climber", "L Winch Stalled", isLeftStalled());
      SmartShuffleboard.put("Climber", "R Winch Stalled", isRightStalled());
      SmartShuffleboard.put("Climber", "L Winch Ever Stalled", leftMotorStall.everStalled());
      SmartShuffleboard.put("Climber", "R Winch Ever Stalled", rightMotorStall.everStalled());
    }
  }
} 
