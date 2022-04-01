package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagPot;
import frc.robot.utils.diag.DiagTalonSrxEncoder;
import frc.robot.utils.diag.DiagTalonSrxSwitch;

public class Hood extends SubsystemBase {
    private WPI_TalonSRX hoodMotor;
    private AnalogPotentiometer potentiometer;

    public Hood(){
       hoodMotor = new WPI_TalonSRX(Constants.HOOD_MOTOR_ID); 
       hoodMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
       hoodMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
       potentiometer = new AnalogPotentiometer(Constants.HOOD_POTENTIOMETER, Constants.HOOD_RANGE_OF_MOTION , Constants.HOOD_STARTING_POINT);

       Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxEncoder("Hood Encoder", 100, hoodMotor));
       Robot.getDiagnostics().addDiagnosable(new DiagPot("Hood Potentiometer", -10, 10, potentiometer));
       Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Hood Forward Switch", hoodMotor, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.FORWARD));
       Robot.getDiagnostics().addDiagnosable(new DiagTalonSrxSwitch("Hood Reverse Switch", hoodMotor, frc.robot.utils.diag.DiagTalonSrxSwitch.Direction.REVERSE));
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

    public double calcPosition(double distance) {
        //THIS IS A PLACEHOLDER METHOD FOR THE TABLES TO GO IN
        return 0;
    }

    public double getEncoder(){
        return hoodMotor.getSelectedSensorPosition();
    }

    public double getPotentiometer(){
        return potentiometer.get();
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartShuffleboard.put("Shooter", "Potentiometer", getPotentiometer());
        SmartShuffleboard.put("Hood", "Potentiometer", getPotentiometer());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}