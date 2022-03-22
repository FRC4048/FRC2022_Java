package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        double new_kP = 0;
        double new_kI = 0;
        double new_kD = 0;
        // This method will be called once per scheduler run
        if (Constants.ENABLE_DEBUG == true) {
            SmartShuffleboard.put("Shooter", "Turret Encoder", getEncoder());
            SmartShuffleboard.put("Shooter", "Left Limit Switch", getLeftSwitch());
            SmartShuffleboard.put("Shooter", "Right Limit Switch", getRightSwitch());
        }
        if (Constants.ENABLE_DEBUG) {
           SmartDashboard.getNumber("turret_kP", new_kP);
           SmartDashboard.getNumber("turret_kI", new_kI);
           SmartDashboard.getNumber("turret_kD", new_kD);
           if (new_kP != turretPID.getP()) {
               turretPID.setP(new_kP);
           }
           if (new_kI != turretPID.getI()) {
               turretPID.setI(new_kI);
           }
           if (new_kD != turretPID.getD()) {
               turretPID.setD(new_kD);
           }
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}