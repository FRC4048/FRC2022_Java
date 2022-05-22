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
import frc.robot.utils.logging.Logging;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

public class TurretSubsystem extends SubsystemBase {
    private WPI_TalonSRX turretMotor;
    private PIDController turretPID;
    private boolean turretLockState;
    

    public TurretSubsystem() {
        turretMotor = new WPI_TalonSRX(Constants.TURRET_MOTOR_ID);
        turretMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        turretMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

        Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxEncoder("Turret  Encoder", 100, turretMotor));
        Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Turret Forward Switch", turretMotor, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.FORWARD));
        Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Turret Reverse Switch", turretMotor, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.REVERSE));

        turretPID = new PIDController(0.075, 0, 0);
        turretPID.setTolerance(2);
    }

    public Logging.LoggingContext loggingContext = new Logging.LoggingContext(this.getClass()) {

        @Override
        protected void addAll() {
            add("Targeting State", Robot.getTargetState().name());
        }
    };

    public PIDController getPID() {
        return turretPID;
    }

    public void setTurret(double speed) {
        turretMotor.set(speed);
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

    public void setTurretLockState(boolean turretLockState){
        this.turretLockState = turretLockState;
    }

    public boolean getTurretLockState() {
        return turretLockState;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (Constants.ENABLE_DEBUG) {
            SmartShuffleboard.put("Shooter", "Turret Encoder", getEncoder());
            SmartShuffleboard.put("Shooter", "Left Limit Switch", getLeftSwitch());
            SmartShuffleboard.put("Shooter", "Right Limit Switch", getRightSwitch());
            SmartDashboard.putData(turretPID);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}