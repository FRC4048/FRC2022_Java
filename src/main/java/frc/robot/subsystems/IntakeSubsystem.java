// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonSRX intakeMotor;
  private Solenoid piston1;
  private Solenoid piston2;
  private DigitalInput intakeSensor;

  
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    //IMPORTANT: these need to be changed from hardcoded values to "Constants.INTAKE_SENSOR" etc once the constants class exists
      intakeMotor = new WPI_TalonSRX(10);
    piston1 = new Solenoid(20, PneumaticsModuleType.CTREPCM, 0);
    piston2 = new Solenoid(20, PneumaticsModuleType.CTREPCM, 3);
    intakeSensor = new DigitalInput(0);

    int TIMEOUT = 100;

    intakeMotor.configNominalOutputForward(0, TIMEOUT);
    intakeMotor.configNominalOutputReverse(0, TIMEOUT);
    intakeMotor.configPeakOutputForward(1, TIMEOUT);
    intakeMotor.configPeakOutputReverse(-1, TIMEOUT); 
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void deployPiston() {
    piston1.set(true);
    piston2.set(true);
  }

  public void retractPiston() {
    piston1.set(false);
    piston2.set(false);
  }

  public void spinMotor(double speed) {
    intakeMotor.set(speed);
  }

  public boolean getIntakeSensor() {
    return !intakeSensor.get();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}