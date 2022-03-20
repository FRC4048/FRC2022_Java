package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagTalonSrxEncoder;
import frc.robot.utils.diag.DiagTalonSrxSwitch;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

public class TurretSubsystem extends SubsystemBase {
    private WPI_TalonSRX turretMotor;
    private PIDController turretPID = new PIDController(Constants.TURRET_kP, Constants.TURRET_kI, Constants.TURRET_kD);

    public TurretSubsystem() {
        turretMotor = new WPI_TalonSRX(Constants.TURRET_MOTOR_ID);
        turretMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        turretMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

        Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxEncoder("Turret  Encoder", 100, turretMotor));
        Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Turret Forward Switch", turretMotor, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.FORWARD));
        Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Turret Reverse Switch", turretMotor, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.REVERSE));
    }

    public void setTurret(double speed) {
        turretMotor.set(speed);
    }

    public void pidSetTurret(double error) {
        turretMotor.set(turretPID.calculate(error, 0));
    }

    public void stopTurret() {
        turretMotor.set(0);
    }

    public boolean getRightSwitch() {
        return turretMotor.getSensorCollection().isRevLimitSwitchClosed();
    }

    public boolean getLeftSwitch() {
        return turretMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public double getEncoder() {
        return turretMotor.getSelectedSensorPosition();
    }

    public void resetEncoder() {
        turretMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (Constants.ENABLE_DEBUG == true) {
            SmartShuffleboard.put("Shooter", "Turret Encoder", getEncoder());
            SmartShuffleboard.put("Shooter", "Left Limit Switch", getLeftSwitch());
            SmartShuffleboard.put("Shooter", "Right Limit Switch", getRightSwitch());
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}