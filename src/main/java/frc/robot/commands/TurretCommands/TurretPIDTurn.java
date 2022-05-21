package frc.robot.commands.TurretCommands;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.LimeLightVision;

public class TurretPIDTurn extends CommandBase {
    private TurretSubsystem turretSubsystem;        
    private LimeLightVision limelight;
    private DoubleSupplier joystickInput;

    public TurretPIDTurn(TurretSubsystem turretSubsystem, LimeLightVision limelight) {
        addRequirements(turretSubsystem);
        this.limelight = limelight;
        this.turretSubsystem = turretSubsystem;
      
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (limelight.hasTarget()) {
            double tx = limelight.getCameraAngles().getTx();
            turretSubsystem.setTurret(turretSubsystem.getPID().calculate(tx, 4));
        }
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