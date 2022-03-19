package frc.robot.commands.HoodCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class MoveHoodToAngle extends CommandBase {
    private Hood hood;
    private Double target;
    private double startTime;
    

    public MoveHoodToAngle(Hood hood, Double angle) {
        this.hood = hood;
        target = angleToTicks(angle);        
        addRequirements(hood);
    }

    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    public void execute() {
        double direction = Math.signum(hood.getPotentiometer() - target);
        hood.setHood(Constants.HOOD_AUTO_MOTOR_SPEED * direction);
    }
        
    @Override
    public void end(boolean interrupted) {
        hood.setHood(0);
    }

    @Override
    public boolean isFinished() {
        return ((target == null) || 
                (Math.abs(hood.getPotentiometer() - target) <= Constants.HOOD_ERROR_THRESHOLD) || 
                ((Timer.getFPGATimestamp() - startTime) >= Constants.HOOD_MOTOR_TIMEOUT));
    }

    private Double angleToTicks(Double angle) {
        if (angle != null) {
            return (106 + angle / (79-60) * (142.5-106));
        } else { return null; }
    }
}
