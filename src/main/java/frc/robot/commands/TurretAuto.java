package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.LimeLightVision;

public class TurretAuto extends CommandBase {
    private TurretSubsystem turretSubsystem;      
    //offset 
    private LimeLightVision vision;
    private boolean positive;
    //Possibly switch these two around
    // private double clockwise = 0.5;    // now in constants
    // private double counterClockwise = -0.5;   // now in constants

    public TurretAuto(TurretSubsystem turretSubsystem, LimeLightVision vision) {
        this.vision = vision;
        addRequirements(turretSubsystem);
        this.turretSubsystem = turretSubsystem;
      
    }
    @Override
    public void initialize() {
       
        if (vision.getCameraAngles().getTx()>=0) {
            positive = true;
        }
        else {
            positive = false;
        }
    }

    @Override
    public void execute() {
        if (positive) {
            turretSubsystem.setTurret(Constants.SHOOTER_COUNTERCLOCKWISE_SPEED);
        }
        else {
            turretSubsystem.setTurret(Constants.SHOOTER_CLOCKWISE_SPEED);
        } 
     
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopTurret();
    }

    @Override
    public boolean isFinished() {
        if (positive){
            if (vision.getCameraAngles().getTx()<=0) {
                return true;
            }
            else {
                return false;
            }
        }
        else{
            if (vision.getCameraAngles().getTx()>=0) {
                return true;
            }
            else {
                return false;
            }
        }
    }

  }