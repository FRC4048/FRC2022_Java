package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.SmartShuffleboard;

public class PidTurnDegrees extends CommandBase{

    private DriveTrain driveTrain;
    private double turnDegrees;
    private double startTime;
    private double startDegrees;
    private double turnSpeed;
    private PIDController pid; 
    private double setPoint;
    private double currentLocation;
    private double error;

    public PidTurnDegrees(DriveTrain driveTrain, double turnDegrees) {
        this.driveTrain = driveTrain;
        this.turnDegrees = turnDegrees;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        this.startTime = Timer.getFPGATimestamp();
        this.startDegrees = driveTrain.getAngle();
        pid = new PIDController(Constants.AUTO_MOVE_TURN_P_CONSTANT, Constants.AUTO_MOVE_TURN_I_CONSTANT, Constants.AUTO_MOVE_TURN_D_CONSTANT);
    }

    @Override
    public void execute() {
        setPoint = (startDegrees + turnDegrees)/180.0;
        currentLocation = driveTrain.getAngle()/180.0;
        error = setPoint - currentLocation;

        if (Math.abs(error)*180 >Constants.AUTO_MOVE_TURN_DEGREES_THRESHOLD){
            turnSpeed = Constants.AUTO_MOVE_TURN_VOLTAGE* Math.signum(error);
        } else {
            turnSpeed = pid.calculate(currentLocation, setPoint);
        }

        if (Constants.ENABLE_DEBUG) {
            SmartShuffleboard.put("Drive","calculated power", turnSpeed);
            SmartShuffleboard.put("Drive","error", error*180.0);
        }

        if (Math.abs(turnSpeed) > Constants.AUTO_MOVE_TURN_VOLTAGE) {
            turnSpeed = Constants.AUTO_MOVE_TURN_VOLTAGE * Math.signum(turnSpeed);
        }

        driveTrain.driveVoltage(-1.0*turnSpeed, turnSpeed);
    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - startTime >= Constants.AUTO_MOVE_TURN_TIMEOUT) {
            return true;
        }
        return (Math.abs(error) * 180 <= Constants.AUTO_MOVE_TURN_THRESHOLD);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(0, 0, false);
    }
    
}
