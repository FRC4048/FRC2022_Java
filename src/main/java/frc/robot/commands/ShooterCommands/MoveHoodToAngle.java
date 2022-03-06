package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class MoveHoodToAngle extends CommandBase {
    private Hood hood;
    private double angle;
    private boolean moveDown;
    private boolean atAngle = false;
    

    public MoveHoodToAngle(Hood hood,double angle) {
        this.hood = hood;
        this.angle = angle;        
        addRequirements(hood);
    }

    public void initialize() {
        if (hood.getPotentiometer() > angle){
            if (hood.getPotentiometer() - angle < 1) {
                atAngle = false;
            }
            moveDown = true;
        }
        else {
            if (hood.getPotentiometer() - angle > -1) {
                atAngle = true;
            }
            moveDown = false;
        }
        
    }

    public void execute() {
        if (atAngle == false){
            if (moveDown == true){
                hood.setHood(Constants.HOOD_MOTOR_SPEED);
            }
            else {
                hood.setHood(Constants.HOOD_MOTOR_SPEED * -1);
            }
        }
    }
        
    @Override
    public void end(boolean interrupted) {
        hood.setHood(0);
    }

    @Override
    public boolean isFinished() {
        
        if (moveDown == true) {
            if (hood.getPotentiometer() <= angle) {
                return true;
            }
            else {
                return false;
            }
        }
        else {
            if (hood.getPotentiometer() >= angle) {
                return true;
            }
            else {
                return false;
            }
        }
    }
}
