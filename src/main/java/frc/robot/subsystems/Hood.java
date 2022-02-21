package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

public class Hood extends SubsystemBase {
    private WPI_TalonSRX hoodMotor;

    public Hood(){
       hoodMotor = new WPI_TalonSRX(Constants.TURRET_MOTOR_ID); 
       hoodMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
       hoodMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }

    public void setHood(double speed){
        hoodMotor.set(speed);
    }

    public void stopHood(){
        hoodMotor.set(0);
    }

    public boolean getRightSwitch(){
        return hoodMotor.getSensorCollection().isRevLimitSwitchClosed();
    }

    public boolean getLeftSwitch(){
        return hoodMotor.getSensorCollection().isFwdLimitSwitchClosed();
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