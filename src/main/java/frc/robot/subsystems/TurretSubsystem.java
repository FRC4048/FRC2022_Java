package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

public class TurretSubsystem extends SubsystemBase {
    private WPI_TalonSRX turretMotor;

    public TurretSubsystem(){
       turretMotor = new WPI_TalonSRX(Constants.TURRET_MOTOR_ID); 
       turretMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
       turretMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
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