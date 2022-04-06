package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.LimeLightVision;

public class ClimberLockTurret extends LoggedCommandBase{

    private TurretSubsystem turret;
    private LimeLightVision limelight;

    private double initTime;

    public ClimberLockTurret (TurretSubsystem turret, LimeLightVision limelight) {
        this.turret = turret;
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        initTime = Timer.getFPGATimestamp();
        limelight.setPipeline(0);
    }

    @Override
    public void execute() {
        turret.setTurret(.3);
    }

    @Override
    public void end(boolean interrupted) {
        turret.setTurret(0);
    }

    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - initTime) >= 3.0 || turret.getLeftSwitch());
    }
    
}
