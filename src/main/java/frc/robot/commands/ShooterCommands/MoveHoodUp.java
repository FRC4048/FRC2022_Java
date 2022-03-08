package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.Hood;

public class MoveHoodUp extends LoggedCommand {
    private Hood hood;
    private double startTimeMillis;
    ;
    

    public MoveHoodUp(Hood hood) {
        this.hood = hood;
        
        addRequirements(hood);
    }

    public void loggedInitialize() {
        startTimeMillis = System.currentTimeMillis();
    }

    public void loggedExecute() {
        hood.setHood(-0.4);
    }

    @Override
    public void loggedEnd(boolean interrupted) {
        hood.setHood(0);
    }

    @Override
    public boolean loggedIsFinished() {
        if (System.currentTimeMillis() - startTimeMillis > 5000){
            return true;
        }
        else {
            return false;
        }
    }   
}
