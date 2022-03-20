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
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagOpticalSensor;
import frc.robot.utils.diag.DiagTalonSrxEncoder;
import frc.robot.utils.logging.Logging;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonSRX intakeMotor;
  private Solenoid piston1;
  private Solenoid piston2;
  private DigitalInput intakeSensor1;
  private DigitalInput intakeSensor2;
  private Solenoid blockPiston;

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR_ID);
    piston1 = new Solenoid(Constants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Constants.INTAKE_SOLENOID_1);
    piston2 = new Solenoid(Constants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Constants.INTAKE_SOLENOID_2);
    intakeSensor1 = new DigitalInput(Constants.INTAKE_SENSOR_ID_1);
    intakeSensor2 = new DigitalInput(Constants.INTAKE_SENSOR_ID_2);
    blockPiston = new Solenoid(Constants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Constants.STOP_SOLENOID_ID);

    int TIMEOUT = 100;

    intakeMotor.configNominalOutputForward(0, TIMEOUT);
    intakeMotor.configNominalOutputReverse(0, TIMEOUT);
    intakeMotor.configPeakOutputForward(1, TIMEOUT);
    intakeMotor.configPeakOutputReverse(-1, TIMEOUT);
    intakeMotor.setNeutralMode(NeutralMode.Brake);

    Robot.getDiagnostics().addDiagnosable(new DiagOpticalSensor("IntakeSensor1", intakeSensor1));
    Robot.getDiagnostics().addDiagnosable(new DiagOpticalSensor("IntakeSensor2", intakeSensor2));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.ENABLE_DEBUG) {
      SmartShuffleboard.put("Intake Sensors", "intake sensor 1", intakeSensor1.get());
      SmartShuffleboard.put("Intake Sensors", "intake sensor 2", intakeSensor2.get());
    }
  }

  public void deployPiston() {
    piston1.set(false);
    piston2.set(false);
  }

  public void retractPiston() {
    piston1.set(true);
    piston2.set(true);
  }
  
  public void togglePiston() {
  piston1.set(!piston1.get());
  piston2.set(!piston2.get());
  }

  public void spinMotor(double speed) {
    intakeMotor.set(speed);
  }

  public boolean isBallInIntake() {
    return !intakeSensor1.get() || !intakeSensor2.get();
  }

  public boolean getIntakeSensor1() {
    return intakeSensor1.get();
  }

  public boolean getIntakeSensor2() {
    return intakeSensor2.get();
  }

  public boolean getPiston1State(){
    return piston1.get();
  }

  public boolean getPiston2State(){
    return piston2.get();
  }

  public boolean getBlockState() {
    return blockPiston.get();
  }

  public void setBlockPiston(boolean newState) {
    blockPiston.set(newState);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public final Logging.LoggingContext loggingContext = new Logging.LoggingContext(this.getClass()) {
    protected void addAll() {
        add("Piston 1 State", getPiston1State());
        add("Piston 2 State", getPiston2State());
        add("Block Piston State", getBlockState());
    }
  };
}
