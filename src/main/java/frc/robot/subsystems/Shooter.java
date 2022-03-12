// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.logging.Logging;

public class Shooter extends SubsystemBase {
  /** Creates a new ClimberElevatorSubsystem. */
  private Solenoid shooterSolenoid;
  private Solenoid stopBallSolenoid;
  private CANSparkMax shooterMotor;
  private DigitalInput elevatorBallSensor;

  public Shooter() {
    //climberSolenoid = new Solenoid(Constants.PCM_CAN_ID, Constants.CLIMBER_PISTON_ID);
    shooterSolenoid = new Solenoid(Constants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Constants.SHOOTER_PISTON_ID);
    stopBallSolenoid = new Solenoid(Constants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Constants.STOP_BALL_SOLENOID);
    shooterMotor = new CANSparkMax(Constants.SHOOTER_MOTOR_ID, MotorType.kBrushless);

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

  public void extendPiston() {
    shooterSolenoid.set(true);
  }

  public void retractPiston() {
    shooterSolenoid.set(false);
  }

  public void extendStopBallPiston() {
    stopBallSolenoid.set(true);
  }

  public void retractStopBallPiston() {
    stopBallSolenoid.set(false);
  }

  public boolean isBallInElevator() {
    return !elevatorBallSensor.get();
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
