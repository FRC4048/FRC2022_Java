package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ManuallyRunIntakeMotor extends LoggedCommand {
    private IntakeSubsystem intakeSubsystem;
    private double motorSpeed;

    public ManuallyRunIntakeMotor(IntakeSubsystem intakeSubsystem, double speed) {
        addRequirements(intakeSubsystem);
        this.intakeSubsystem = intakeSubsystem;
        this.motorSpeed = speed;
    }
    
    public void loggedInitialize() {}

    public void loggedExecute() {
        intakeSubsystem.spinMotor(motorSpeed);
    }

    @Override
    public void loggedEnd(boolean interrupted) {}

    @Override
    public boolean loggedIsFinished(){
        return true;
    }
}
