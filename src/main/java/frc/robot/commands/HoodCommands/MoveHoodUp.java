package frc.robot.commands.HoodCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.subsystems.Hood;

public class MoveHoodUp extends LoggedCommandBase {
    private Hood hood;
    private double startTimeMillis;
    ;
    

    public MoveHoodUp(Hood hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    public void initialize() {
        startTimeMillis = System.currentTimeMillis();
    }

    public void execute() {
        hood.setHood(-0.4);
    }

    @Override
    public void end(boolean interrupted) {
        hood.setHood(0);
    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() - startTimeMillis > 5000){
            return true;
        }
        else {
            return false;
        }
    }   
}
