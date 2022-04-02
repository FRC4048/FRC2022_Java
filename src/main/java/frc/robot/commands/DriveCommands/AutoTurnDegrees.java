package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoTurnDegrees extends CommandBase{

    private DriveTrain driveTrain;
    private double turnDegrees;
    private double startTime;
    private double startDegrees;
    private double turnSpeed = Constants.AUTO_MOVE_TURN_SPEED;

    public AutoTurnDegrees(DriveTrain driveTrain, double angleToTurn) {
        this.driveTrain = driveTrain;
        this.turnDegrees = angleToTurn;
    }

    @Override
    public void initialize() {
        this.startTime = Timer.getFPGATimestamp();
        this.startDegrees = driveTrain.getAngle();
    }

    @Override
    public void execute() {
        if (turnDegrees > 0) {
            driveTrain.drive(turnSpeed, -turnSpeed, false);
        } else if (turnDegrees < 0) {
            driveTrain.drive(-turnSpeed, turnSpeed, false);
        }
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
        double turnedSoFar = driveTrain.getAngle() - startDegrees;
        if (Math.abs(turnedSoFar) >= Math.abs(turnDegrees)) {
            return true;
        }
        return false;
    }
    
}
