package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.diag.DiagTalonSrxSwitch;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

public class TurretSubsystem extends SubsystemBase {
    private WPI_TalonSRX turretMotor;

    public TurretSubsystem(){
       turretMotor = new WPI_TalonSRX(Constants.TURRET_MOTOR_ID); 
       turretMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
       turretMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

        Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Turret Forward Switch", turretMotor, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.FORWARD));
        Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Turret_Reverse_Switch", turretMotor, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.REVERSE));
    }

    public void setTurret(double speed){
        turretMotor.set(speed);
    }

    public void stopTurret(){
        turretMotor.set(0);
    }

    public boolean getRightSwitch(){
        return turretMotor.getSensorCollection().isRevLimitSwitchClosed();
    }

    public boolean getLeftSwitch(){
        return turretMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }




    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}