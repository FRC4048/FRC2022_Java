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

    public PidTurnDegrees(DriveTrain driveTrain, double turnDegrees) {
        this.driveTrain = driveTrain;
        this.turnDegrees = turnDegrees;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        this.startTime = Timer.getFPGATimestamp();
        this.startDegrees = driveTrain.getAngle();
        pid = new PIDController(0.005, 0.005, 0.002);
    }

    @Override
    public void execute() {
        double rawError = (startDegrees + turnDegrees - driveTrain.getAngle());
        double error = Math.abs(rawError);
        double direction = Math.signum(rawError);
        turnSpeed= pid.calculate(driveTrain.getAngle(), startDegrees + turnDegrees);
        if (Math.abs(turnSpeed)>0.7){
            turnSpeed=0.7 * Math.signum(turnSpeed);
        }
        driveTrain.drive(-1.0 * turnSpeed, turnSpeed, false);
        /*if (error > Constants.AUTO_MOVE_TURN_SLOWDOWN_ERROR) {
            turnSpeed = Constants.AUTO_MOVE_TURN_MAX_SPEED;
        }
        else {
            turnSpeed = error/Constants.AUTO_MOVE_TURN_SLOWDOWN_ERROR * (Constants.AUTO_MOVE_TURN_MAX_SPEED - Constants.AUTO_MOVE_TURN_MIN_SPEED) + Constants.AUTO_MOVE_TURN_MIN_SPEED;
        }
        driveTrain.drive(-1.0 * direction * turnSpeed, direction * turnSpeed, false);
        */
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(0, 0, false);
    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - startTime >= 3) {
            return true;
        }
        return false;

       // double error = Math.abs(startDegrees + turnDegrees - driveTrain.getAngle());
       // return (error <= 0.1);
        //return (error <= Constants.AUTO_MOVE_TURN_THRESHOLD);
    }
    
}
