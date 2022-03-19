package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.limelight.LimeLightVision;

public class TurretAuto extends LoggedCommandBase {
    private TurretSubsystem turretSubsystem;
    // offset
    private LimeLightVision limeLight;
    // Adjust speed as necessary when testing
    // private double clockwise = 0.5; // now in constants
    // private double counterClockwise = -0.5; // now in constants
    // bad readings
    private int badReadings;
    private double speed;
    private double initTime;

    public TurretAuto(TurretSubsystem turretSubsystem, LimeLightVision limeLight) {
        this.limeLight = limeLight;
        addRequirements(turretSubsystem);
        this.turretSubsystem = turretSubsystem;
addLog(limeLight.getCameraAngles().getTx());
    }

    @Override
    public void initialize() {
        initTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (limeLight.getCameraAngles() != null) {
            speed = Math.abs(limeLight.getCameraAngles().getTx())/Constants.TURRET_MAX_DIFFERENCE*(Constants.TURRET_SPEED-Constants.TURRET_MIN_SPEED)+Constants.TURRET_MIN_SPEED;
            badReadings = 0;
            turretSubsystem.setTurret(-1 * Math.signum(limeLight.getCameraAngles().getTx()) * speed);
        } else {
            badReadings++;
        }
        SmartShuffleboard.put("Shooter", "Turret Speed", speed);

    }

    @Override
    public void end(boolean interrupted) {
        addLog(limeLight.getCameraAngles().getTx());
        turretSubsystem.stopTurret();
    }

    @Override
    public boolean isFinished() {

        if (limeLight.getCameraAngles() != null) {
            if (Math.abs(limeLight.getCameraAngles().getTx()) <= Constants.TURRET_AUTO_ALIGN_TRESHOLD) {
                return true;
            }
        }
        if (badReadings >= Constants.TURRET_AUTO_BAD_READINGS_TRESHOLD) {
            return true;
        }
        if ((Timer.getFPGATimestamp() - initTime) >= Constants.TURRET_AUTO_TIMEOUT) {
            return true;
        }
        return false;
    }
}