package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.DiagSparkMaxEncoder;
import frc.robot.utils.logging.Logging;


public class DriveTrain extends SubsystemBase {
    public CANSparkMax left1;
    public CANSparkMax left2;
    private CANSparkMax right1;
    private CANSparkMax right2;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private SlewRateLimiter linearFilter;
    private SlewRateLimiter angularFilter;
    private DifferentialDriveKinematics kinematics;
    private DifferentialDriveWheelSpeeds wheelSpeeds;
    private ChassisSpeeds chassisSpeeds;
    

    private double p;
    private double i;
    private double d;
    
    private final ADIS16470_IMU imu;

    public static final double maxSpeed = 3.0; // meters per second
    public static final double maxAngularSpeed = 2 * Math.PI; // one rotation per second

    private static final double trackWidth = 0.5461; // meters
    //private static final int encoderResolution = 42 * 3; //Ticks per revolution ; for NEO it's 42*3

    // Tune PIDs
    private final PIDController leftPIDController = new PIDController(1, 0, 0);
    private final PIDController rightPIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);

    private DifferentialDriveOdometry odometry;
    private Field2d fieldMap;

    public DriveTrain(){

        left1 = new CANSparkMax(Constants.DRIVE_LEFT1_ID, MotorType.kBrushless);
        left2 = new CANSparkMax(Constants.DRIVE_LEFT2_ID, MotorType.kBrushless);
        right1 = new CANSparkMax(Constants.DRIVE_RIGHT1_ID, MotorType.kBrushless);
        right2 = new CANSparkMax(Constants.DRIVE_RIGHT2_ID, MotorType.kBrushless);

        linearFilter = new SlewRateLimiter(4);
        angularFilter = new SlewRateLimiter(6);

        kinematics = new DifferentialDriveKinematics(trackWidth);
        wheelSpeeds = new DifferentialDriveWheelSpeeds();
        chassisSpeeds = new ChassisSpeeds();

        imu = new ADIS16470_IMU();
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()), new Pose2d( 5.0, 5.0, new Rotation2d()));
        fieldMap = new Field2d();
        fieldMap.setRobotPose(odometry.getPoseMeters());

        resetGyro();

        left1.restoreFactoryDefaults();
        left2.restoreFactoryDefaults();

        right1.restoreFactoryDefaults();
        right2.restoreFactoryDefaults();

        leftEncoder = left1.getEncoder();
        rightEncoder = right1.getEncoder();

        leftEncoder.setPositionConversionFactor(2 * Constants.WHEEL_RADIUS * Math.PI / Constants.CHASSIS_GEAR_RATIO);
        rightEncoder.setPositionConversionFactor(2 * Constants.WHEEL_RADIUS * Math.PI / Constants.CHASSIS_GEAR_RATIO);

        left2.follow(left1);
        right2.follow(right1);

        right1.setInverted(true);
        right2.setInverted(true);

        left1.setIdleMode(IdleMode.kBrake);
        left2.setIdleMode(IdleMode.kBrake);
        right1.setIdleMode(IdleMode.kBrake);
        right2.setIdleMode(IdleMode.kBrake);
        
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        
        Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("Left Drive Encoder", 10, left1));
        Robot.getDiagnostics().addDiagnosable(new DiagSparkMaxEncoder("Right Drive Encoder", 10, right1));
        // TODO: Are there diags for the IMU?

        SmartDashboard.putNumber("P Left", 0.0001);
        SmartDashboard.putNumber("I Left", 0);
        SmartDashboard.putNumber("D Left", 0);
    }

    public void drive(double speedLeft, double speedRight, boolean isSquared) {

        wheelSpeeds.leftMetersPerSecond = speedLeft;
        wheelSpeeds.rightMetersPerSecond = speedRight;

        SmartShuffleboard.put("DriveTrain", "LeftWheel", "LeftWheelSpeed", wheelSpeeds.leftMetersPerSecond);

        //Use kinematics library to filter drive speed
        chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
        double linear = linearFilter.calculate(chassisSpeeds.vxMetersPerSecond);
        double angular = angularFilter.calculate(chassisSpeeds.omegaRadiansPerSecond);
        ChassisSpeeds filteredChassisSpeeds = new ChassisSpeeds(linear, 0.0, angular); 
        wheelSpeeds = kinematics.toWheelSpeeds(filteredChassisSpeeds);
        SmartShuffleboard.put("DriveTrain", "LeftWheel", "LeftWheelFilteredSpeed", wheelSpeeds.leftMetersPerSecond);
        
        //Set feedforwards and PIDs
        final double leftFeedforward = feedforward.calculate(wheelSpeeds.leftMetersPerSecond);
        final double rightFeedforward = feedforward.calculate(wheelSpeeds.rightMetersPerSecond);
        final double leftPIDOutput = leftPIDController.calculate(leftEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond);
        final double rightPIDOutput = rightPIDController.calculate(rightEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond);
        
        //Set wheel voltage
        left1.setVoltage(leftPIDOutput + leftFeedforward);
        right1.setVoltage(rightPIDOutput + rightFeedforward);

        SmartShuffleboard.put("DriveTrain", "Left", "Left", leftPIDOutput + leftFeedforward);
        SmartShuffleboard.put("DriveTrain", "Right", "Right", rightPIDOutput + rightFeedforward);

        //TODO move this to filter joystick inputs
        /* if(isSquared) {
            speedLeft = Math.signum(speedLeft) * Math.pow(speedLeft, 2);
            speedRight = Math.signum(speedRight) * Math.pow(speedRight, 2);
          } */
          // driveTrain.tankDrive(speedLeft, speedRight);
          //The joysticks are inverted so inverting this makes it drive correctly.
        }

    public void driveVoltage(double voltageLeft, double voltageRight){
        left1.setVoltage(voltageLeft);
        right1.setVoltage(voltageRight);
    }
      /**
   * Resets the Gyro
   */
    public void resetGyro() {
        imu.reset();
        imu.calibrate();
    }

    public void resetEncoders() {
        
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

         p = SmartDashboard.getNumber("P Left", 1);
         i = SmartDashboard.getNumber("I Left", 0);
         d = SmartDashboard.getNumber("D Left", 0);
 
         leftPIDController.setP(p);
         leftPIDController.setI(i);
         leftPIDController.setD(d);
 
         rightPIDController.setP(p);
         rightPIDController.setI(i);
         rightPIDController.setD(d);
         odometry.update(Rotation2d.fromDegrees(imu.getAngle()), getLeftEncoder(), getRightEncoder());
         fieldMap.setRobotPose(odometry.getPoseMeters());
         SmartDashboard.putData("Field", fieldMap);
    }

    public double getLeftEncoder(){
        return leftEncoder.getPosition();
    }

    public double getRightEncoder(){
        return rightEncoder.getPosition();
    }

    public Logging.LoggingContext loggingContext = new Logging.LoggingContext(this.getClass()) {

        @Override
        protected void addAll() {
            add("Left Ecnoder", getLeftEncoder());
            add("Right Encoder", getRightEncoder());
            add("AccelX", imu.getAccelX());
            add("AccelY", imu.getAccelY());
            add("Angle", imu.getAngle());
            add("Pose2dX", odometry.getPoseMeters().getX());
            add("Pose2dY", odometry.getPoseMeters().getY());
        }
    };

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
