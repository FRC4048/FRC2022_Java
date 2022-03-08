package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.Hood;

public class MoveHoodToAngle extends LoggedCommand {
    private Hood hood;
    private double angle;
    private boolean moveDown;
    private boolean atAngle = false;
    

    public MoveHoodToAngle(Hood hood,double angle) {
        this.hood = hood;
        this.angle = angle;        
        addRequirements(hood);
    }

    public void loggedInitialize() {
        addLog(angle);

        if (hood.getPotentiometer() > angle){
            if (hood.getPotentiometer() - angle < Constants.HOOD_MARGIN_OF_ERROR) {
                atAngle = false;
            }
            moveDown = true;
        }
        else {
            if (hood.getPotentiometer() - angle > -Constants.HOOD_MARGIN_OF_ERROR) {
                atAngle = true;
            }
            moveDown = false;
        }
        
    }

    public void loggedExecute() {
        if (atAngle == false){
            if (moveDown == true){
                hood.setHood(Constants.HOOD_MOTOR_SPEED);
            }
            else {
                hood.setHood(-Constants.HOOD_MOTOR_SPEED);
            }
        }
    }
        
    @Override
    public void loggedEnd(boolean interrupted) {
        addLog(hood.getPotentiometer() - angle);
        hood.setHood(0);
    }

    @Override
    public boolean loggedIsFinished() {
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
