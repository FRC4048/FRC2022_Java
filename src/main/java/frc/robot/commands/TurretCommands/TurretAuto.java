package frc.robot.commands.TurretCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.LimeLightVision;

public class TurretAuto extends CommandBase {
    private TurretSubsystem turretSubsystem;      
    //offset 
    private LimeLightVision limeLight;
    private boolean positive;
    private double speed;
    //Adjust speed as necessary when testing
    // private double clockwise = 0.5;    // now in constants
    // private double counterClockwise = -0.5;   // now in constants

    public TurretAuto(TurretSubsystem turretSubsystem, LimeLightVision limeLight) {
        this.limeLight = limeLight;
        addRequirements(turretSubsystem);
        this.turretSubsystem = turretSubsystem;
      
    }
    @Override
    public void initialize() {
       if (limeLight.getCameraAngles()!=null){
            if (limeLight.getCameraAngles().getTx()>=0) {
                positive = true;
            }
            else {
                positive = false;
            }
        }
    }

    @Override
    public void execute() {
        speed = Math.abs(limeLight.getCameraAngles().getTx())/Constants.TURRET_MAX_DIFFERENCE*(Constants.TURRET_MAX_SPEED-Constants.TURRET_MIN_SPEED)+Constants.TURRET_MIN_SPEED;
        if (positive) {
            turretSubsystem.setTurret(speed);
        }
        else {
            turretSubsystem.setTurret(-speed);
        } 
     
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopTurret();
    }

    @Override
    public boolean isFinished() {
        if (limeLight.getCameraAngles() == null){
            return true;
        }
        else{
            if (positive){
                if (limeLight.getCameraAngles().getTx()<=0) {
                    return true;
                }
                else {
                    return false;
                }
            }
            else{
                if (limeLight.getCameraAngles().getTx()>=0) {
                    return true;
                }
                else {
                    return false;
                }
            }
        }
    }

  }