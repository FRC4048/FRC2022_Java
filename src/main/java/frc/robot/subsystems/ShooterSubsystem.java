// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagSparkMaxEncoder;
import frc.robot.utils.logging.Logging;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ClimberElevatorSubsystem. */
  private Solenoid shooterSolenoid;
  private CANSparkMax shooterMotor;
  private boolean isRunning;
  private Solenoid blockPiston;

  public ShooterSubsystem() {
    //climberSolenoid = new Solenoid(Constants.PCM_CAN_ID, Constants.CLIMBER_PISTON_ID);
    shooterSolenoid = new Solenoid(Constants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Constants.SHOOTER_PISTON_ID);
    shooterMotor = new CANSparkMax(Constants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
    blockPiston = new Solenoid(Constants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Constants.STOP_SOLENOID_ID);
    isRunning = false;

    SmartDashboard.putNumber("DesiredSpeed", .85);

    Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("Shooter Encoder", 100, shooterMotor));

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

  public void setBlockPiston(boolean newState) {
    blockPiston.set(newState);
  }

  public void extendPiston() {
    shooterSolenoid.set(true);
  }

  public void retractPiston() {
    shooterSolenoid.set(false);
  }

  public boolean getBlockState() {
    return blockPiston.get();
  }

  public boolean getPistonState() {
    return shooterSolenoid.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.ENABLE_DEBUG == true){
      SmartShuffleboard.put("Shooter", "Data", "Piston State", getPistonState());
      SmartShuffleboard.put("Shooter", "Data", "Shooter RPM", getEncoder().getVelocity());
      SmartShuffleboard.put("Shooter", "Data", "Block Piston", getBlockState());
    }
  }
  
  public final Logging.LoggingContext loggingContext = new Logging.LoggingContext(this.getClass()) {
      protected void addAll() {
          add("Piston State", getPistonState());
          add("Shooter Speed", getShooterSpeed());
      }
  };
}
