package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMUConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonSRX left1;
  private WPI_TalonSRX right1;
  private WPI_TalonSRX left2;
  private WPI_TalonSRX right2;
  private DifferentialDrive driveTrain;
  private Encoder leftEncoder;
  private Encoder rightEncoder;
  // private AHRS navX;
  private PigeonIMU gyro;
  private Solenoid gearSolenoid;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    left1 = new WPI_TalonSRX(6);
    left2 = new WPI_TalonSRX(3);
    right1 = new WPI_TalonSRX(7);
    right2 = new WPI_TalonSRX(8);

    // navX = new AHRS(I2C.Port.kMXP);
    left2.set(ControlMode.Follower, 6);
    right2.set(ControlMode.Follower, 7);
    
    // right1.setInverted(true);
    // right2.setInverted(true);    
    left1.setInverted(true);
    left2.setInverted(true);

    left1.setNeutralMode(NeutralMode.Brake);
    left2.setNeutralMode(NeutralMode.Brake);
    right1.setNeutralMode(NeutralMode.Brake);
    right2.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Drives the robot by a speed between -1 and +1
   * 
   * @param speedLeft
   * @param speedRight
   */
  public void drive(double speedLeft, double speedRight, boolean isSquared){
    if(isSquared) {
      speedLeft = Math.signum(speedLeft) * Math.pow(speedLeft, 2);
      speedRight = Math.signum(speedRight) * Math.pow(speedRight, 2);
    }
    // driveTrain.tankDrive(speedLeft, speedRight);
    //The joysticks are inverted so inverting this makes it drive correctly.
    left1.set(ControlMode.PercentOutput, speedLeft);
    right1.set(ControlMode.PercentOutput, speedRight);
  }
}
