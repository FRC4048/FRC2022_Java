package frc.robot.commands.turretcommand;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretManualCommand extends CommandBase {
    private TurretSubsystem turretSubsystem;        
    private DoubleSupplier joystickInput;

    public TurretManualCommand(TurretSubsystem turretSubsystem, DoubleSupplier joystickXAxis) {
        joystickInput=joystickXAxis;
        addRequirements(turretSubsystem);
        this.turretSubsystem = turretSubsystem;
      
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //dont know if the input should be negative
      turretSubsystem.setTurret((joystickInput.getAsDouble() * Constants.TURRETSPIN_SCALEFACTOR));
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopTurret();
    }

    @Override
    public boolean isFinished() {
        return false;

    }


  }