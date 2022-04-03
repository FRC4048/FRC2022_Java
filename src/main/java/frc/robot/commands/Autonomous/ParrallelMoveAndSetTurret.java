package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DriveCommands.MoveDistance;
import frc.robot.commands.IntakeCommand.IntakeSequence;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ParrallelMoveAndSetTurret extends ParallelCommandGroup {
    public ParrallelMoveAndSetTurret(DriveTrain driveTrain, double speed, double distanceInches, TurretSubsystem turretSubsystem, double turretSpeed, TurretSubsystem TurretSubsystem) {
        addCommands(
            new MoveDistance(driveTrain, speed, distanceInches),
            new AutoSetShootingPosition(turretSubsystem, speed, 425)
        );
    }
}
