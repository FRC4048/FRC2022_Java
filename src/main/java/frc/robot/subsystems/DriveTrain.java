package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagSparkMaxEncoder;

public class DriveTrain extends SubsystemBase {
    public CANSparkMax left1;
    public CANSparkMax left2;
    private CANSparkMax right1;
    private CANSparkMax right2;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private DifferentialDriveOdometry odometry;
    private Field2d fieldMap = new Field2d();

    private SlewRateLimiter linearFilter;
    private SlewRateLimiter angularFilter;
    private DifferentialDriveKinematics kinematics;
    private DifferentialDriveWheelSpeeds wheelSpeeds;
    private ChassisSpeeds chassisSpeeds;
    
    private final ADIS16470_IMU imu;

    public DriveTrain(){
        left1 = new CANSparkMax(Constants.DRIVE_LEFT1_ID, MotorType.kBrushless);
        left2 = new CANSparkMax(Constants.DRIVE_LEFT2_ID, MotorType.kBrushless);
        right1 = new CANSparkMax(Constants.DRIVE_RIGHT1_ID, MotorType.kBrushless);
        right2 = new CANSparkMax(Constants.DRIVE_RIGHT2_ID, MotorType.kBrushless);

        linearFilter = new SlewRateLimiter(4);
        angularFilter = new SlewRateLimiter(6);

        kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22));
        wheelSpeeds = new DifferentialDriveWheelSpeeds();
        chassisSpeeds = new ChassisSpeeds();

        imu = new ADIS16470_IMU();

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()), new Pose2d( 0.0, 0.0, new Rotation2d()));
        fieldMap = new Field2d();

        left1.restoreFactoryDefaults();
        left2.restoreFactoryDefaults();

        right1.restoreFactoryDefaults();
        right2.restoreFactoryDefaults();

        leftEncoder = left1.getEncoder();
        rightEncoder = right1.getEncoder();

        leftEncoder.setPositionConversionFactor(Constants.WHEEL_DIAMETER * Math.PI / Constants.GEAR_RATIO);
        rightEncoder.setPositionConversionFactor(Constants.WHEEL_DIAMETER * Math.PI / Constants.GEAR_RATIO);

        left2.follow(left1);
        right2.follow(right1);

        right1.setInverted(true);
        right2.setInverted(true);

        left1.setIdleMode(IdleMode.kBrake);
        left2.setIdleMode(IdleMode.kBrake);
        right1.setIdleMode(IdleMode.kBrake);
        right2.setIdleMode(IdleMode.kBrake);

        resetOdometry();
        

        Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("Left Drive Encoder", 10, left1));
        Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("Right Drive Encoder", 10, right1));
        // TODO: Are there diags for the IMU?
    }

    public void drive(double speedLeft, double speedRight, boolean isSquared) {

        wheelSpeeds.leftMetersPerSecond = speedLeft;
        wheelSpeeds.rightMetersPerSecond = speedRight;
        chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
        double linear = linearFilter.calculate(chassisSpeeds.vxMetersPerSecond);
        double angular = angularFilter.calculate(chassisSpeeds.omegaRadiansPerSecond);
        ChassisSpeeds filteredChassisSpeeds = new ChassisSpeeds(linear, 0.0, angular); 
        wheelSpeeds = kinematics.toWheelSpeeds(filteredChassisSpeeds);
        left1.set(wheelSpeeds.leftMetersPerSecond);
        right1.set(wheelSpeeds.rightMetersPerSecond);
        
      
        /*if(isSquared) {
            speedLeft = Math.signum(speedLeft) * Math.pow(speedLeft, 2);
            speedRight = Math.signum(speedRight) * Math.pow(speedRight, 2);
          }

          */

          // driveTrain.tankDrive(speedLeft, speedRight);
          //The joysticks are inverted so inverting this makes it drive correctly.
    }

      /**
   * Resets the Gyro
   */
    public void resetGyro() {
        imu.reset();
        imu.calibrate();
    }

    public void resetOdometry() {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
        resetGyro();
    }

      /**
   * Gets the angle of the robot
   * 
   * @return angle of robot between -180-180
   */
    public double getAngle() {
        return imu.getAngle();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (Constants.ENABLE_DEBUG) {
            SmartShuffleboard.put("Drive", "Encoders", "L", getLeftEncoder());
            SmartShuffleboard.put("Drive", "Encoders", "R", getRightEncoder());
            SmartShuffleboard.put("Drive", "Gyro", "Angle", getAngle());
            SmartShuffleboard.put("Drive", "Gyro", "Raw Angle", imu.getAngle());
            SmartShuffleboard.put("Drive", "Gyro", "X Comp Angle", imu.getXComplementaryAngle());
            SmartShuffleboard.put("Drive", "Gyro", "Y Comp Angle", imu.getYComplementaryAngle());
            SmartShuffleboard.put("Drive", "Gyro", "X filtered acceleration angle", imu.getXFilteredAccelAngle());
            SmartShuffleboard.put("Drive", "Gyro", "Y filtered acceleration angle", imu.getYFilteredAccelAngle());
         }
        odometry.update(Rotation2d.fromDegrees(-imu.getAngle()), getLeftEncoder(), getRightEncoder());
        fieldMap.setRobotPose(odometry.getPoseMeters());
        SmartShuffleboard.put("Drive", "Field", fieldMap);
    }

    public double getLeftEncoder(){
        return leftEncoder.getPosition();
    }

    public double getRightEncoder(){
        return rightEncoder.getPosition();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}