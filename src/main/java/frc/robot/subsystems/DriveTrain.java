package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SmartShuffleboard;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonSRX left1;
  private WPI_TalonSRX right1;
  private WPI_TalonSRX left2;
  private WPI_TalonSRX right2;
  private DifferentialDrive driveTrain;
  private Encoder leftEncoder;
  private Encoder rightEncoder;
  // private AHRS navX;
  private ADIS16470_IMU gyro;
  private Solenoid gearSolenoid;
  private SlewRateLimiter linearFilter;
  private SlewRateLimiter angularFilter;
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveWheelSpeeds wheelSpeeds;
  private ChassisSpeeds chassisSpeeds;
  private DifferentialDriveOdometry odometry;
  private Field2d field;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    left1 = new WPI_TalonSRX(6);
    left2 = new WPI_TalonSRX(3);
    right1 = new WPI_TalonSRX(7);
    right2 = new WPI_TalonSRX(8);
    
    chassisSpeeds = new ChassisSpeeds();
    wheelSpeeds = new DifferentialDriveWheelSpeeds();
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(19.25));

    leftEncoder = new Encoder(Constants.DRIVE_ENCODER_LEFT_ID[0], Constants.DRIVE_ENCODER_LEFT_ID[1], true);
    rightEncoder = new Encoder(Constants.DRIVE_ENCODER_RIGHT_ID[0], Constants.DRIVE_ENCODER_RIGHT_ID[1]);

    leftEncoder.setDistancePerPulse(1.0 / 214);
    rightEncoder.setDistancePerPulse(1.0 / 214);

    linearFilter = new SlewRateLimiter(4);
    angularFilter = new SlewRateLimiter(8);

    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(19.25));

    gyro = new ADIS16470_IMU();
    gyro.reset();
    gyro.calibrate();

    odometry = new DifferentialDriveOdometry(new Rotation2d(gyro.getAngle()));
    field = new Field2d(); 
    SmartDashboard.putData("Field", field);

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
    

    addToShuffleboard();
  }

  private void addToShuffleboard() {
    /* SmartShuffleboard.put("Drive", "angle", gyro.getAngle());
    SmartShuffleboard.put("Drive", "Gyro", "X filtered acceleration angle", gyro.getXFilteredAccelAngle());
    SmartShuffleboard.put("Drive", "Gyro", "Y filtered acceleration angle", gyro.getYFilteredAccelAngle());
    SmartShuffleboard.put("Drive", "Gyro", "Z acceleration angle", gyro.getAccelZ());
    SmartShuffleboard.put("Drive", "Gyro", "X acceleration angle", gyro.getAccelX());
    SmartShuffleboard.put("Drive", "Gyro", "Y acceleration angle", gyro.getAccelY());*/
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
    wheelSpeeds.leftMetersPerSecond = speedLeft;
    wheelSpeeds.rightMetersPerSecond = speedRight;
    chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
    double linear = linearFilter.calculate(chassisSpeeds.vxMetersPerSecond);
    double angular = angularFilter.calculate(chassisSpeeds.omegaRadiansPerSecond);
    ChassisSpeeds filteredChassisSpeeds = new ChassisSpeeds(linear, 0.0, angular); 
    wheelSpeeds = kinematics.toWheelSpeeds(filteredChassisSpeeds);
    left1.set(ControlMode.PercentOutput, wheelSpeeds.leftMetersPerSecond);
    right1.set(ControlMode.PercentOutput, wheelSpeeds.rightMetersPerSecond);
    //left1.set(ControlMode.PercentOutput, (lFilter.calculate(speedLeft)) * 0.75);
    //right1.set(ControlMode.PercentOutput, (rFilter.calculate(speedRight)) * 0.75);\
    SmartShuffleboard.put("Drive", "leftEncoder", leftEncoder.get());
    SmartShuffleboard.put("Drive", "rightEncoder", rightEncoder.get());
    SmartShuffleboard.put("Drive", "limited left speed", linearFilter.calculate(speedLeft));
    SmartShuffleboard.put("Drive", "limited right speed", angularFilter.calculate(speedRight));
    SmartShuffleboard.put("Drive", "left speed", speedLeft);
    SmartShuffleboard.put("Drive", "right speed", speedRight);
  }

  public double getAngle() {
    return gyro.getAngle();
  }

  public ADIS16470_IMU getGyro(){
    return gyro;
  }

  public void resetRightEncoder() {
    rightEncoder.reset();
  }

  public void resetLeftEncoder() {
    leftEncoder.reset();  
  }
  public void periodic(){
    odometry.update(Rotation2d.fromDegrees(getAngle()), leftEncoder.getDistance(), rightEncoder.getDistance());
    field.setRobotPose(odometry.getPoseMeters());
    SmartShuffleboard.put("Drive", "leftEncoder", leftEncoder.get());
    SmartShuffleboard.put("Drive", "rightEncoder", rightEncoder.get());
  }
}
