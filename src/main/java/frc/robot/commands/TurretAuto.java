package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.LimeLightVision;

public class TurretAuto extends LoggedCommand {
    private TurretSubsystem turretSubsystem;      
    //offset 
    private LimeLightVision limeLight;
    private boolean positive;
    //Adjust speed as necessary when testing
    // private double clockwise = 0.5;    // now in constants
    // private double counterClockwise = -0.5;   // now in constants

    public TurretAuto(TurretSubsystem turretSubsystem, LimeLightVision limeLight) {
        this.limeLight = limeLight;
        addRequirements(turretSubsystem);
        this.turretSubsystem = turretSubsystem;
        addLog(limeLight.getCameraAngles().getTx());
    }

    @Override
    public void loggedInitialize() {
       
        if (limeLight.getCameraAngles().getTx()>=0) {
            positive = true;
        }
        else {
            positive = false;
        }
    }

    @Override
    public void loggedExecute() {
        if (positive) {
            turretSubsystem.setTurret(Constants.SHOOTER_CLOCKWISE_SPEED);
        }
        else {
            turretSubsystem.setTurret(Constants.SHOOTER_COUNTERCLOCKWISE_SPEED);
        } 
     
    }

    @Override
    public void loggedEnd(boolean interrupted) {
        turretSubsystem.stopTurret();
    }

    @Override
    public boolean loggedIsFinished() {
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