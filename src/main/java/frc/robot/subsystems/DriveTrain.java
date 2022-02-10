package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants2022Robot;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.Constants2022Test;

public class DriveTrain extends SubsystemBase {
    public CANSparkMax left1;
    public CANSparkMax left2;
    private CANSparkMax right1;
    private CANSparkMax right2;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    public DriveTrain(){
        left1 = new CANSparkMax(Constants2022Test.MOTOR_LEFT1_ID, MotorType.kBrushless);
        left2 = new CANSparkMax(Constants2022Test.MOTOR_LEFT2_ID, MotorType.kBrushless);
        //right1 = new CANSparkMax(Constants2022Robot.MOTOR_RIGHT1_ID, MotorType.kBrushless);
        //right2 = new CANSparkMax(Constants2022Robot.MOTOR_RIGHT2_ID, MotorType.kBrushless);

        leftEncoder = left1.getEncoder();
        //rightEncoder = right1.getEncoder();

        left2.follow(left1);
        //right2.follow(right1);

        left1.setInverted(true);
        left2.setInverted(true);

        left1.setIdleMode(IdleMode.kBrake);
        left2.setIdleMode(IdleMode.kBrake);
        //right1.setIdleMode(IdleMode.kBrake);
        //right2.setIdleMode(IdleMode.kBrake);
    }

    public void drive(double speedLeft, /*double speedRight,*/ boolean isSquared) {
        if(isSquared) {
            speedLeft = Math.signum(speedLeft) * Math.pow(speedLeft, 2);
            //speedRight = Math.signum(speedRight) * Math.pow(speedRight, 2);
          }
          // driveTrain.tankDrive(speedLeft, speedRight);
          //The joysticks are inverted so inverting this makes it drive correctly.
          left1.set(speedLeft);
          //right1.set(speedRight);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (Constants2022Robot.ENABLE_DEBUG) {
            SmartShuffleboard.put("Drive", "Encoders", "L", getLeftEncoder());
            /*SmartShuffleboard.put("Drive", "Encoders", "R", getRightEncoder());*/
         }
    }

    public double getLeftEncoder(){
        return leftEncoder.getPosition();
    }

    /*public double getRightEncoder(){
        return rightEncoder.getPosition();
    }*/

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}