// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberArmSubsystem extends SubsystemBase {
  /** Creates a new ClimberArmSubsystem. */
  private TalonSRX leftArm, rightArm;
  private Solenoid leftPiston, rightPiston;
  
  public ClimberArmSubsystem() {
    leftArm = new TalonSRX(Constants.CLIMBER_LEFT_MOTOR_ID);
    rightArm = new TalonSRX(Constants.CLIMBER_RIGHT_MOTOR_ID);
    leftPiston = new Solenoid(Constants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Constants.CLIMBER_LEFT_PISTON_ID);
    rightPiston = new Solenoid(Constants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Constants.CLIMBER_RIGHT_PISTON_ID);
    
    leftArm.setNeutralMode(NeutralMode.Brake);
    leftArm.setNeutralMode(NeutralMode.Brake);
  }

  public void setLeftArmSpeed(double speed) {
    leftArm.set(ControlMode.PercentOutput, speed);
  }

  public void setRightArmSpeed(double speed) {
    rightArm.set(ControlMode.PercentOutput, speed);
  }

  public void movePiston(boolean state) {
    leftPiston.set(state);
    rightPiston.set(state);
  }

  public void stopLeftArm() {
    leftArm.set(ControlMode.PercentOutput, 0);
  }

  public void stopRightArm() {
    rightArm.set(ControlMode.PercentOutput, 0);
  }

  public double getLeftArmVoltage() {
    return leftArm.getBusVoltage();
  }

  public double getRightArmVoltage() {
    return rightArm.getBusVoltage();
  }

  public void setLeftArmMode(NeutralMode mode) {
    leftArm.setNeutralMode(mode);
  }

  public void setRightArmMode(NeutralMode mode) {
    rightArm.setNeutralMode(mode);
  }


  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
