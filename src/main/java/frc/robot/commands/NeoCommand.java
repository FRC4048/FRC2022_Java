package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoSubsystem;

public class NeoCommand extends CommandBase{

    private NeoSubsystem neosubSystem;

    public NeoCommand(NeoSubsystem subsystem) {
        this.neosubSystem = subsystem;
        addRequirements(subsystem);
    }

    public void execute() {
        double speed = SmartDashboard.getNumber("neospeed", 0.0);
        this.neosubSystem.setSpeed(speed);
    }

      // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

