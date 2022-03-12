// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.logging.Logging;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ClimberElevatorSubsystem. */
  private Solenoid shooterSolenoid;
  private CANSparkMax shooterMotor;
  private boolean isRunning;

  public ShooterSubsystem() {
    //climberSolenoid = new Solenoid(Constants.PCM_CAN_ID, Constants.CLIMBER_PISTON_ID);
    shooterSolenoid = new Solenoid(Constants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Constants.SHOOTER_PISTON_ID);
    shooterMotor = new CANSparkMax(Constants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
    isRunning = false;

    shooterMotor.setIdleMode(IdleMode.kCoast);
    shooterMotor.setInverted(false);
    
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  public void stopShooter() {
    shooterMotor.set(0);
  }

  public double getShooterSpeed() {
    return shooterMotor.get();
  }

  public RelativeEncoder getEncoder() {
    return shooterMotor.getEncoder();
  }

  public boolean isRunning() {
    return isRunning;
  }

  public void setRunning(boolean state) {
    isRunning = state;
  }

  public void extendPiston() {
    shooterSolenoid.set(true);
  }

  public void retractPiston() {
    shooterSolenoid.set(false);
  }

  public boolean getPistonState() {
    return shooterSolenoid.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.ENABLE_DEBUG == true){
      SmartShuffleboard.put("Shooter", "Data", "Piston State", getPistonState());;
      SmartShuffleboard.put("Shooter", "Data", "Shooter RPM", getEncoder().getVelocity());
    }
  }
  
    public final Logging.LoggingContext loggingContext = new Logging.LoggingContext(this.getClass()) {
      protected void addAll() {
          add("Piston State", getPistonState());
          add("Shooter Speed", getShooterSpeed());
      }
  };
  
}
