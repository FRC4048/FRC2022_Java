package frc.robot.commands.IntakeCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class ManuallyRunIntakeMotor extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private double motorSpeed;
    private double startTime;

    public ManuallyRunIntakeMotor(IntakeSubsystem intakeSubsystem, double speed) {
        addRequirements(intakeSubsystem);
        this.intakeSubsystem = intakeSubsystem;
        this.motorSpeed = speed;
    }
    
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    public void execute() {
        intakeSubsystem.spinMotor(motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.spinMotor(0);
    }

    @Override
    public boolean isFinished(){
        if ((Timer.getFPGATimestamp() - startTime) >=  Constants.INTAKE_MOTOR_TIMEOUT) {
            return true;
        } 
        else {
            return false;
        }
    }
}
