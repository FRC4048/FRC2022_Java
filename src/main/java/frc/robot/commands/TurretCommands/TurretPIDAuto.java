package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.CameraAngles;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.limelight.LimeLightVision;

public class TurretPIDAuto extends LoggedCommandBase {
    private TurretSubsystem turretSubsystem;
    // offset
    private LimeLightVision limeLight;
    // Adjust speed as necessary when testing
    // private double clockwise = 0.5; // now in constants
    // private double counterClockwise = -0.5; // now in constants
    // bad readings
    private double initTime;

    public TurretPIDAuto(TurretSubsystem turretSubsystem, LimeLightVision limeLight) {
        this.limeLight = limeLight;
        addRequirements(turretSubsystem);
        this.turretSubsystem = turretSubsystem;
        CameraAngles angles = limeLight.getCameraAngles();
        if (angles != null) {
            addLog(angles.getTx());
        }
    }

    @Override
    public void initialize() {
        initTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (limeLight.hasTarget()) {
            turretSubsystem.pidSetTurret(limeLight.getCameraAngles().getTx());
        }
    }

    @Override
    public void end(boolean interrupted) {
        CameraAngles angles = limeLight.getCameraAngles();
        if (angles != null) {addLog(angles.getTx());}
        turretSubsystem.stopTurret();
    }

    @Override
    public boolean isFinished() {
        if (limeLight.hasTarget()) {
            if (Math.abs(limeLight.getCameraAngles().getTx()) <= Constants.TURRET_AUTO_ALIGN_TRESHOLD) {
                return true;
            }
        }
        if ((Timer.getFPGATimestamp() - initTime) >= Constants.TURRET_AUTO_TIMEOUT) {
            return true;
        }
        return false;
    }
}
