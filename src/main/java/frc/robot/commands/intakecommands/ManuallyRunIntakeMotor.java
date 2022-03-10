package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ManuallyRunIntakeMotor extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private double motorSpeed;

    public ManuallyRunIntakeMotor(IntakeSubsystem intakeSubsystem, double speed) {
        addRequirements(intakeSubsystem);
        this.intakeSubsystem = intakeSubsystem;
        this.motorSpeed = speed;
    }
    
    public void initialize() {}

    public void execute() {
        intakeSubsystem.spinMotor(motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished(){
        return true;
    }
}
