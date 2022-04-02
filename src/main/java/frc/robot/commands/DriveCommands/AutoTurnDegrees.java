package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.SmartShuffleboard;

public class AutoTurnDegrees extends CommandBase{

    private DriveTrain driveTrain;
    private double turnDegrees;
    private double startTime;
    private double startDegrees;
    private double turnSpeed;

    public AutoTurnDegrees(DriveTrain driveTrain, double angleToTurn) {
        this.driveTrain = driveTrain;
        this.turnDegrees = angleToTurn;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        this.startTime = Timer.getFPGATimestamp();
        this.startDegrees = driveTrain.getAngle();
    }

    @Override
    public void execute() {
        double rawError = (startDegrees + turnDegrees - driveTrain.getAngle());
        double error = Math.abs(rawError);
        double direction = Math.signum(rawError);
        SmartShuffleboard.put("Drive", "Turn error", error);
        if (error > Constants.AUTO_MOVE_TURN_SLOWDOWN_ERROR) {
            turnSpeed = Constants.AUTO_MOVE_TURN_MAX_SPEED;
        }
        else {
            turnSpeed = error/Constants.AUTO_MOVE_TURN_SLOWDOWN_ERROR * (Constants.AUTO_MOVE_TURN_MAX_SPEED - Constants.AUTO_MOVE_TURN_MIN_SPEED) + Constants.AUTO_MOVE_TURN_MIN_SPEED;
        }
        SmartShuffleboard.put("Drive", "Turn speed", turnSpeed);
        driveTrain.drive(-1.0 * direction * turnSpeed, direction * turnSpeed, false);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(0, 0, false);
    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - startTime >= Constants.AUTO_MOVE_TURN_TIMEOUT) {
            return true;
        }
        double error = Math.abs(startDegrees + turnDegrees - driveTrain.getAngle());
        SmartShuffleboard.put("Drive", "Turn error", error);
        return (error <= Constants.AUTO_MOVE_TURN_THRESHOLD);
    }
    
}
