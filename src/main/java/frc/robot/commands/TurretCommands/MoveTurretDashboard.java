package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class MoveTurretDashboard extends CommandBase {
    private TurretSubsystem turretSubsystem;        
    private double startTime;
    public enum Direction {
        RIGHT,LEFT
    }
    private Direction direction;

    public MoveTurretDashboard(TurretSubsystem turretSubsystem, Direction direction) {
        addRequirements(turretSubsystem);
        this.direction = direction;
        this.turretSubsystem = turretSubsystem;
    }
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        switch (direction) {
            case LEFT:
                turretSubsystem.setTurret(-0.5);
            break;
            case RIGHT:
                turretSubsystem.setTurret(0.5);
            break;
        }
       
      
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopTurret();
    }

    @Override
    public boolean isFinished() {
        if ((Timer.getFPGATimestamp() - startTime) >= Constants.TURRETSPIN_COMMAND_TIMEOUT) {
            return true;
        }
        else {
            return false;
        }
    }
  }