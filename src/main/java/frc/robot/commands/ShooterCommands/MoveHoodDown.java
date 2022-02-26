package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class MoveHoodDown extends CommandBase {
    private Hood hood;
    ;
    

    public MoveHoodDown(Hood hood) {
        this.hood = hood;
        
        addRequirements(hood);
    }

    public void initialize() {

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
        return false;
    }
}
