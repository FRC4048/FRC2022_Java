package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.DriveCommands.MoveDistance;
import frc.robot.commands.HoodCommands.MoveHoodToAngle;
import frc.robot.commands.IntakeCommand.IntakeSequence;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ParallelMoveAndTurretResetAndIntake extends ParallelCommandGroup {
    public ParallelMoveAndTurretResetAndIntake(DriveTrain driveTrain, double speed, double distanceMeters, TurretSubsystem turretSubsystem, double turretSpeed, IntakeSubsystem intakeSubsystem, Hood hood) {
        addCommands(
            new MoveDistance(driveTrain, speed, distanceMeters),
            new AutoSetShootingPosition(turretSubsystem, turretSpeed, Constants.AUTO_TURRET_CENTER_ANGLE),
            new IntakeSequence(intakeSubsystem, 4)
        );
    }
}
