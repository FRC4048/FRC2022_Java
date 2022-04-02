package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.CameraAngles;
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
    private double speed;
    private double initTime;
    private static double CAMERA_OFFSET = 2;

    public TurretAuto(TurretSubsystem turretSubsystem, LimeLightVision limeLight) {
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
            double tx = limeLight.getCameraAngles().getTx();
            if (tx > (CAMERA_OFFSET + Constants.TURRET_ERROR_THRESHOLD) ||tx > (CAMERA_OFFSET - Constants.TURRET_ERROR_THRESHOLD) ) {
                speed = Constants.TURRET_FAST_SPEED; 
            }
            else {
                speed = Constants.TURRET_SLOW_SPEED;
            }
            if (tx > CAMERA_OFFSET) {
                turretSubsystem.setTurret(speed * -1);
            } else if (tx < CAMERA_OFFSET) {
                turretSubsystem.setTurret(speed);
            }
            //turretSubsystem.setTurret(-1 * Math.signum((limeLight.getCameraAngles().getTx()) + 2) * speed);
        }
        else {  
            turretSubsystem.setTurret(0);
        }
        if (Constants.ENABLE_DEBUG) {
            SmartShuffleboard.put("Shooter", "Turret Speed", speed);
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
            if ((limeLight.getCameraAngles().getTx() >= CAMERA_OFFSET - Constants.TURRET_AUTO_ALIGN_TRESHOLD) && 
            (limeLight.getCameraAngles().getTx() <= CAMERA_OFFSET + Constants.TURRET_AUTO_ALIGN_TRESHOLD)) {
                return true;
            }
        }
        if ((Timer.getFPGATimestamp() - initTime) >= Constants.TURRET_AUTO_TIMEOUT) {
            return true;
        }
        return false;
    }
}