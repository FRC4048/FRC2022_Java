package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.SmartShuffleboard;

public class TurretCommand extends CommandBase {
    private TurretSubsystem turretSubsystem;
    private XboxController xboxController;

    public TurretCommand(TurretSubsystem turretSubsystem, XboxController xboxController) {
        addRequirements(turretSubsystem);
        this.turretSubsystem = turretSubsystem;
        this.xboxController = xboxController;
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      turretSubsystem.setTurret((xboxController.getLeftX() * Constants.TURRETSPIN_SPEED));
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopTurret();
    }

    @Override
    public boolean isFinished() {
        return false; //TODO: figure out the actual logic for when command will stop

    }


  }