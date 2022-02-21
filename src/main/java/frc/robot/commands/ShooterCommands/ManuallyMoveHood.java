package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class ManuallyMoveHood extends CommandBase {
    private Hood hood;
    private DoubleSupplier rightJoystickY;
    

    public ManuallyMoveHood(Hood hood, DoubleSupplier rightJoystickY) {
        this.hood = hood;
        this.rightJoystickY = rightJoystickY;
        addRequirements(hood);
    }

    public void initialize() {

    }

    public void execute() {
        if(Math.abs(rightJoystickY.getAsDouble()) < 0.2){
            hood.setHood(0);
        }
        hood.setHood(rightJoystickY.getAsDouble()*Constants.HOOD_MOTOR_SPEED);
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
