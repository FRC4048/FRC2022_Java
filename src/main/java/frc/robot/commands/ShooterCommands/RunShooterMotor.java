package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterMotor extends CommandBase {
    private ShooterSubsystem shooter;

    public RunShooterMotor(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (shooter.getVelocity() == 0) {
            shooter.setShooterSpeed(0);
        } else { shooter.setShooterRPM(shooter.getVelocity()); }
    }

    @Override
    public void end(boolean interrupted) { shooter.setShooterRPM(0); }

    @Override
    public boolean isFinished() { return false; }
}
