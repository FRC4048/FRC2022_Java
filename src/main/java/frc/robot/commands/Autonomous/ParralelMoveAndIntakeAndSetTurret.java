package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DriveCommands.MoveDistance;
import frc.robot.commands.IntakeCommand.IntakeSequence;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ParralelMoveAndIntakeAndSetTurret extends ParallelCommandGroup {
    public ParralelMoveAndIntakeAndSetTurret(DriveTrain driveTrain, double speed, double distanceMeters, TurretSubsystem turretSubsystem, double turretSpeed, IntakeSubsystem intakeSubsystem, Hood hood, TurretSubsystem TurretSubsystem) {
        addCommands(
            new MoveDistance(driveTrain, speed, distanceMeters),
            new IntakeSequence(intakeSubsystem)
            //new AutoSetTurretPosition(turretSubsystem, speed, 425)
        );
    }
}
