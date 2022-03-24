package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoTurnDegrees extends CommandBase{

    private DriveTrain driveTrain;
    private double TurnDegrees;
    private double startTime;
    private double startDegrees;
    private double TURN_SPEED = 0.2;

    public AutoTurnDegrees(DriveTrain driveTrain, double angleToTurn) {
        this.driveTrain = driveTrain;
        this.TurnDegrees = angleToTurn;
    }

    @Override
    public void initialize() {
        this.startTime = Timer.getFPGATimestamp();
        this.startDegrees = driveTrain.getAngle();
    }

    @Override
    public void execute() {
        if (TurnDegrees > 0) {
            driveTrain.drive(TURN_SPEED, -TURN_SPEED, false);
        } else if (TurnDegrees < 0) {
            driveTrain.drive(-TURN_SPEED, TURN_SPEED, false);
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
        if (Math.abs(turnedSoFar) >= Math.abs(TurnDegrees)) {
            return true;
        }
        return false;
    }
    
}
