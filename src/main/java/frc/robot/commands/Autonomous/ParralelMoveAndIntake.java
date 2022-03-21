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
    public ParralelMoveAndIntake(DriveTrain driveTrain, double speed, double distanceInches, TurretSubsystem turretSubsystem, double turretSpeed, IntakeSubsystem intakeSubsystem, Hood hood) {
        addCommands(
            new MoveDistance(driveTrain, speed, distanceInches),
            new IntakeSequence(intakeSubsystem)
            //we will eventually need to move the turret here to make sure the goal is in frame after this move. 
            //new SetTurretPosition(TurretSubsystem, speed)
        );
    }
}
