package frc.robot.commands.TurretCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.LimeLightVision;

public class TurretAuto extends LoggedCommandBase {
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
    public void initialize() {
       
        if (limeLight.getCameraAngles().getTx()>=0) {
            positive = true;
        }
        else {
            positive = false;
        }
    }

    @Override
    public void execute() {
        if (positive) {
            turretSubsystem.setTurret(Constants.TURRET_CLOCKWISE_SPEED);
        }
        else {
            turretSubsystem.setTurret(Constants.TURRET_COUNTERCLOCKWISE_SPEED);
        } 
     
    }

    @Override
    public void end(boolean interrupted) {
        addLog(limeLight.getCameraAngles().getTx());
        turretSubsystem.stopTurret();
    }

    @Override
    public boolean isFinished() {

        if (limeLight.getCameraAngles() == null){
            return true;
        }

        else{

            if (positive){
                if (limeLight.getCameraAngles().getTx() <= 0) {
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