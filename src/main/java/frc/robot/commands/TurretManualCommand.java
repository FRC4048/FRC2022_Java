package frc.robot.commands;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretManualCommand extends LoggedCommand {
    private TurretSubsystem turretSubsystem;        
    private DoubleSupplier joystickInput;

    public TurretManualCommand(TurretSubsystem turretSubsystem, DoubleSupplier joystickXAxis) {
        joystickInput = joystickXAxis;
        addRequirements(turretSubsystem);
        this.turretSubsystem = turretSubsystem;
      
    }
    @Override
    public void loggedInitialize() {
    }

    @Override
    public void loggedExecute() {
        //dont know if the input should be negative
      turretSubsystem.setTurret((joystickInput.getAsDouble() * Constants.TURRETSPIN_SCALEFACTOR));
    }

    @Override
    public void loggedEnd(boolean interrupted) {
        turretSubsystem.stopTurret();
    }

    @Override
    public boolean loggedIsFinished() {
        return false;

    }
}