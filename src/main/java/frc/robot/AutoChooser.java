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
import frc.robot.commands.Autonomous.OneShotSequenceMiddle;
import frc.robot.commands.Autonomous.ThreeShotSequenceRight;
import frc.robot.commands.Autonomous.TwoShotSequenceRight;
import frc.robot.commands.DriveCommands.Drive;
import frc.robot.commands.IntakeCommand.IntakeSequence;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.LimeLightVision;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;

/**
 * Add your docs here.
 */
public class AutoChooser {
    private SendableChooser<Action> actionChooser;
    // private NetworkTableEntry delayEntry;
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private DriveTrain driveTrain;
    private TurretSubsystem turretSubsystem;
    private LimeLightVision limeLightVision;
    private Hood hood;

    // all actions driver choose at beginning of match
    enum Action {
        THREE_SHOT_RIGHT, TWO_SHOT_RIGHT, TWO_SHOT_LEFT, ONE_SHOT, CROSS_LINE, DO_NOTHING;
    }


    public AutoChooser(IntakeSubsystem intakeSubsystem, DriveTrain driveTrain, ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem, LimeLightVision limeLightVision, Hood hood) {
        actionChooser = new SendableChooser<Action>();
        this.intakeSubsystem = intakeSubsystem;
        this.driveTrain = driveTrain;
        this.shooterSubsystem = shooterSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.limeLightVision = limeLightVision;
        this.hood = hood;
    }

    public void addOptions() {
        
         
         actionChooser.setDefaultOption(Action.TWO_SHOT_RIGHT.name(), Action.TWO_SHOT_RIGHT);
         actionChooser.addOption(Action.THREE_SHOT_RIGHT.name(), Action.THREE_SHOT_RIGHT); 
         actionChooser.addOption(Action.TWO_SHOT_LEFT.name(), Action.TWO_SHOT_LEFT);
         actionChooser.addOption(Action.ONE_SHOT.name(), Action.ONE_SHOT);
         actionChooser.addOption(Action.CROSS_LINE.name(), Action.CROSS_LINE);
         actionChooser.addOption(Action.DO_NOTHING.name(), Action.DO_NOTHING);
         

    }

    public void initialize() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
        tab.add("Autonomous Action", actionChooser);
    }

    
     public Action getAction(){
        if(actionChooser.getSelected() != null){
            return actionChooser.getSelected();
        } else{
            return Action.TWO_SHOT_RIGHT;
        }
     }

    public Command getAutonomousCommand(Action a) {
        if (a == Action.TWO_SHOT_RIGHT) {return new TwoShotSequenceRight(turretSubsystem,
            Constants.AUTO_TURRET_SPEED, intakeSubsystem, driveTrain, Constants.AUTO_MOVE_SPEED,
            Constants.AUTO_DISTANCE_INCHES, shooterSubsystem, limeLightVision, hood);}
        else if (a == Action.THREE_SHOT_RIGHT
        ) {return new ThreeShotSequenceRight(turretSubsystem, 
            Constants.AUTO_TURRET_SPEED,  intakeSubsystem, driveTrain, Constants.AUTO_MOVE_SPEED, 
            Constants.AUTO_DISTANCE_INCHES, shooterSubsystem, limeLightVision, hood);}
        else if (a == Action.TWO_SHOT_LEFT) {return new TwoShotSequenceLeft(turretSubsystem, 
            Constants.AUTO_TURRET_SPEED,  intakeSubsystem, driveTrain, Constants.AUTO_MOVE_SPEED, 
            Constants.AUTO_DISTANCE_INCHES, shooterSubsystem, limeLightVision, hood);}
        else if (a == Action.ONE_SHOT) {return new OneShotSequenceMiddle(turretSubsystem, intakeSubsystem, 
            driveTrain, shooterSubsystem, limeLightVision, hood, Constants.AUTO_MOVE_SPEED, 
            Constants.AUTO_DISTANCE_INCHES);}
        else if (a == Action.CROSS_LINE) {return new CrossTheLineSequence(driveTrain, turretSubsystem);}
        else if (a == Action.DO_NOTHING) {return new DoNothingSequence(turretSubsystem);}
        return new WaitCommand(0);
    }
}
