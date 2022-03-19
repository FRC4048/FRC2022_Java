/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Autonomous.CrossTheLineSequence;
import frc.robot.commands.Autonomous.DoNothingSequence;
import frc.robot.commands.Autonomous.TwoShotSequenceLeft;
import frc.robot.commands.Autonomous.TwoShotSequenceMiddle;
import frc.robot.commands.Autonomous.TwoShotSequenceRight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;

/**
 * Add your docs here.
 */
public class AutoChooser {
    private SendableChooser<Position> positionChooser;
    private SendableChooser<Action> actionChooser;
    // private NetworkTableEntry delayEntry;
    AutoCommand autonomousCommand;

    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final DriveTrain driveTrain = new DriveTrain();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();

    // position at beginning of match
    enum Position {
        MIDDLE, LEFT, RIGHT;
    }

    // all actions driver choose at beginning of match
    enum Action {
        TWO_SHOT, DO_NOTHING, CROSS_LINE;
    }

    // all commmands during autonomous
    enum AutoCommand {
        A_MIDDLE, B_MIDDLE, C_MIDDLE, A_LEFT, B_LEFT, C_LEFT, A_RIGHT, B_RIGHT, C_RIGHT;
    }
    // replace ABC options once we decide what we are doing in auto

    public AutoChooser() {
        positionChooser = new SendableChooser<Position>();
        actionChooser = new SendableChooser<Action>();
        // AutoCommand autonomousCommand = getAutonomousCommand(getPosition(),
        // getAction());
        // ^^ see line 92
    }

    public void addOptions() {
        
         positionChooser.addOption(Position.LEFT.name(), Position.LEFT);
         positionChooser.addOption(Position.MIDDLE.name(), Position.MIDDLE);
         positionChooser.addOption(Position.RIGHT.name(), Position.RIGHT);
         actionChooser.setDefaultOption(Action.TWO_SHOT.name(), Action.TWO_SHOT);
         actionChooser.addOption(Action.DO_NOTHING.name(), Action.DO_NOTHING);
         actionChooser.addOption(Action.CROSS_LINE.name(), Action.CROSS_LINE);
         

    }

    public void initialize() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
        tab.add("Autonomous Position", positionChooser);
        tab.add("Autonomous Action", actionChooser);
        // delayEntry = tab.add("Delay", 0).getEntry();
    }

    
     public Action getAction(){
     if(actionChooser.getSelected() != null){
        return actionChooser.getSelected();
     } else{
        return Action.CROSS_LINE;
     }
     }
     

    public Position getPosition() {
        if (positionChooser.getSelected() != null) {
            return positionChooser.getSelected();
        } else {
            return Position.MIDDLE;
        }
    }

    /*
     * Needs to be updated
     * public int getDelay(){
     * int delay = delayEntry.getNumber(0).intValue();
     * if(autonomousCommand == AutoCommand.DO_NOTHING
     * || autonomousCommand == AutoCommand.RIGHT_PICKUP
     * || autonomousCommand == AutoCommand.CROSS_LINE){
     * return 0;
     * }
     * if(delay <= 6 && delay > 0){
     * return delay;
     * }else{
     * return 0;
     * }
     * }
     */

    // Had to take this stuff out because there is not an AutoCommands class, could
    // be reimplemented later

    public Command getAutonomousCommand(Position p, Action a) {

        // TEMPORARY UNTIL AUTO COMMANDS IMPLEMENTED
        if (a == Action.TWO_SHOT) {
            if (p == Position.LEFT) {
                TwoShotSequenceLeft TwoShotSequenceLeft = new TwoShotSequenceLeft(turretSubsystem,
                        Constants.AUTO_TURRET_SPEED, intakeSubsystem, driveTrain, Constants.AUTO_MOVE_SPEED,
                        Constants.AUTO_DISTANCE_INCHES, shooterSubsystem);
                return TwoShotSequenceLeft;
            } else if (p == Position.MIDDLE) {
                TwoShotSequenceMiddle TwoShotSequenceMiddle = new TwoShotSequenceMiddle(turretSubsystem,
                        intakeSubsystem, driveTrain, shooterSubsystem);
                return TwoShotSequenceMiddle;
            } else if (p == Position.RIGHT) {
                TwoShotSequenceRight TwoShotSequenceRight = new TwoShotSequenceRight(turretSubsystem,
                        Constants.AUTO_TURRET_SPEED, intakeSubsystem, driveTrain, Constants.AUTO_MOVE_SPEED,
                        Constants.AUTO_DISTANCE_INCHES, shooterSubsystem);
                return TwoShotSequenceRight;
            }
        } else if (a == Action.DO_NOTHING) {
            if (p == Position.LEFT) {
                DoNothingSequence DoNothingSequence = new DoNothingSequence();
                return DoNothingSequence;
            } else if (p == Position.MIDDLE) {
                DoNothingSequence DoNothingSequence = new DoNothingSequence();
                return DoNothingSequence;
            } else if (p == Position.RIGHT) {
                DoNothingSequence DoNothingSequence = new DoNothingSequence();
                return DoNothingSequence;
            }
        } 
        else if (a == Action.CROSS_LINE) {
            if (p == Position.LEFT) {
                CrossTheLineSequence CrossTheLineSequence = new CrossTheLineSequence(driveTrain);
                return CrossTheLineSequence;
            } else if (p == Position.MIDDLE) {
                CrossTheLineSequence CrossTheLineSequence = new CrossTheLineSequence(driveTrain);
                return CrossTheLineSequence;
            } else if (p == Position.RIGHT) {
                CrossTheLineSequence CrossTheLineSequence = new CrossTheLineSequence(driveTrain);
                return CrossTheLineSequence;
            }
        }
        return new WaitCommand(0);
    }
}
