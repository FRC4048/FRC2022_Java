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

public class MoveAndMoveHood extends ParallelCommandGroup {
    public MoveAndMoveHood(DriveTrain driveTrain, double speed, double distanceInches, Hood hood) {
        addCommands(
            new MoveDistance(driveTrain, speed, distanceInches),
            new MoveHoodToAngle(hood, 112.0)
        );
    }
}
