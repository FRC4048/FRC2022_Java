package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.LimeLightVision;

public class ClimberInfiniteLockTurret extends LoggedCommandBase{

    private TurretSubsystem turret;
    private LimeLightVision limelight;

    public ClimberInfiniteLockTurret (TurretSubsystem turret, LimeLightVision limelight) {
        this.turret = turret;
        this.limelight = limelight;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(0);
    }

    @Override
    public void execute() {
        turret.setTurret(.15);
    }

    @Override
    public void end(boolean interrupted) {
        turret.setTurret(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
