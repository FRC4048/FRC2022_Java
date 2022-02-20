package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretAuto extends CommandBase {
    private TurretSubsystem turretSubsystem;      
    //offset 
    private DoubleSupplier offset;
    private boolean positive;
    //Possibly switch these two around
    private double clockwise = 0.5;
    private double counterClockwise = -0.5;
    public TurretAuto(TurretSubsystem turretSubsystem, DoubleSupplier horizontalOffset) {
        this.offset = horizontalOffset;
        addRequirements(turretSubsystem);
        this.turretSubsystem = turretSubsystem;
      
    }
    @Override
    public void initialize() {
       
        if (offset.getAsDouble()>=0) {
            positive = true;
        }
        else {
            positive = false;
        }
    }

    @Override
    public void execute() {
        if (positive) {
            turretSubsystem.setTurret(counterClockwise);
        }
        else {
            turretSubsystem.setTurret(clockwise);
        } 
     
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopTurret();
    }

    @Override
    public boolean isFinished() {
        if (positive){
            if (offset.getAsDouble()<=0) {
                return true;
            }
            else {
                return false;
            }
        }
        else{
            if (offset.getAsDouble()>=0) {
                return true;
            }
            else {
                return false;
            }
        }
    }


  }