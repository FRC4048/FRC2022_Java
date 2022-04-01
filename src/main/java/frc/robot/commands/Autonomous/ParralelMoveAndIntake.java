package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetTurretPosition;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.DriveCommands.MoveDistance;
import frc.robot.commands.HoodCommands.MoveHoodToAngle;
import frc.robot.commands.IntakeCommand.IntakeSequence;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ParralelMoveAndIntake extends ParallelCommandGroup {
    public ParralelMoveAndIntake(DriveTrain driveTrain, double speed, double distanceInches, TurretSubsystem turretSubsystem, double turretSpeed, IntakeSubsystem intakeSubsystem, Hood hood, TurretSubsystem TurretSubsystem) {
        addCommands(
            new MoveDistance(driveTrain, speed, distanceInches),
            new IntakeSequence(intakeSubsystem),
            //temporary angle value, needs to be tested.
            new AutoSetTurretPosition(TurretSubsystem, 20, speed)
        );
    }
    public ParralelMoveAndIntake(DriveTrain driveTrain, double speed, double distanceInches, TurretSubsystem turretSubsystem, double turretSpeed, IntakeSubsystem intakeSubsystem, Hood hood, TurretSubsystem TurretSubsystem, int timeOut) {
        addCommands(
            new MoveDistance(driveTrain, speed, distanceInches),
            new IntakeSequence(intakeSubsystem, timeOut),
            //temporary angle value, needs to be tested.
            new AutoSetTurretPosition(TurretSubsystem, 20, speed)
        );
    }
}
