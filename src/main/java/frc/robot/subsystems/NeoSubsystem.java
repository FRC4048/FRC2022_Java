package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NeoSubsystem extends SubsystemBase{

    private CANSparkMax motor;

    public NeoSubsystem() {
      // THIS MAY NEED TO CHANGE ON A NEW ROBOT
        this.motor = new CANSparkMax(44, MotorType.kBrushless);
    }

    public void setSpeed (double spd) {
        motor.set(spd);
    }

    public double getSpeed() {
        return motor.get();
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
