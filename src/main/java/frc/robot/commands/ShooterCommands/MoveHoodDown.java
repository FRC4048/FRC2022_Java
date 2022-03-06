package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class MoveHoodDown extends CommandBase {
    private Hood hood;
    private double startTimeMillis;
    ;
    

    public MoveHoodDown(Hood hood) {
        this.hood = hood;
        
        addRequirements(hood);
    }

    public void initialize() {
        startTimeMillis = System.currentTimeMillis();
    }

    public void execute() {
        hood.setHood(0.4);
    }

    @Override
    public void end(boolean interrupted) {
        hood.setHood(0);
    }

    @Override
    public boolean isFinished() {
        if (startTimeMillis - System.currentTimeMillis() > 5000){
            return true;
        }
        else {
            return false;
        }
    }
}
