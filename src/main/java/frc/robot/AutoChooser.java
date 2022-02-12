/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;

/**
 * Add your docs here.
 */
public class AutoChooser {
    private SendableChooser<Position> positionChooser;
    private SendableChooser<Action> actionChooser;
    //private NetworkTableEntry delayEntry;
    AutoCommand autonomousCommand;
    //position at beginning of match
    enum Position{
        MIDDLE, LEFT, RIGHT;
    }
    //all actions driver choose at beginning of match
    enum Action {
        DO_NOTHING, CROSS_LINE, ACTION_C ;
    }
    //all commmands during autonomous
    enum AutoCommand {
        CROSS_LINE, DO_NOTHING, C_MIDDLE, A_LEFT, B_LEFT, C_LEFT, A_RIGHT, B_RIGHT, C_RIGHT;
    }
    //replace ABC options once we decide what we are doing in auto
 
    public AutoChooser(){
        positionChooser = new SendableChooser<Position>();
        actionChooser = new SendableChooser<Action>();
        //AutoCommand autonomousCommand = getAutonomousCommand(getPosition(), getAction());
        // ^^ see line 92 
    }
    public void addOptions(){
        positionChooser.addOption(Position.LEFT.name(), Position.LEFT);
        positionChooser.addOption(Position.MIDDLE.name(), Position.MIDDLE);
        positionChooser.addOption(Position.RIGHT.name(), Position.RIGHT); 
        actionChooser.setDefaultOption(Action.DO_NOTHING.name(), Action.DO_NOTHING);
        actionChooser.addOption(Action.CROSS_LINE.name(), Action.CROSS_LINE);
        actionChooser.addOption(Action.ACTION_C.name(), Action.ACTION_C);

    }
    public void initialize() {
        ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
        tab.add("Autonomous Position", positionChooser);
        tab.add("Autonomous Action", actionChooser);
        //delayEntry = tab.add("Delay", 0).getEntry();
    }

    public Action getAction(){
        if(actionChooser.getSelected() != null){
            return actionChooser.getSelected();
        } else{
            return Action.ACTION_C;
        }
    }

    public Position getPosition(){
        if(positionChooser.getSelected() != null){
            return positionChooser.getSelected();
        } else{
            return Position.MIDDLE;
        }
    }

    /* Needs to be updated 
    public int getDelay(){
        int delay = delayEntry.getNumber(0).intValue();
        if(autonomousCommand == AutoCommand.DO_NOTHING
            || autonomousCommand == AutoCommand.RIGHT_PICKUP 
            || autonomousCommand == AutoCommand.CROSS_LINE){
            return 0;
        }
        if(delay <= 6 && delay > 0){
            return delay;
        }else{
            return 0;
        }
    }
    */




     //Had to take this stuff out because there is not an AutoCommands class, could be reimplemented later

    public AutoCommand getAutonomousCommand( Position p, Action a){

        if (a== Action.DO_NOTHING){
            return AutoCommand.DO_NOTHING;
        }
        else{
            if(a==Action.CROSS_LINE){
                return AutoCommand.CROSS_LINE;
            }
            else{
                return AutoCommand.DO_NOTHING;
            }
            
        }
    }
    
       /* if (a == Action.ACTION_A){
            if (p == Position.LEFT){
                return AutoCommand.A_LEFT;
            }   
            else if (p == Position.MIDDLE){
                return AutoCommand.A_MIDDLE;
            }
            else if (p == Position.RIGHT){
                return AutoCommand.A_RIGHT;
            }
        }
        else if (a == Action.ACTION_B){
            if (p == Position.LEFT){
                return AutoCommand.B_LEFT;
            }   
            else if (p == Position.MIDDLE){
                return AutoCommand.B_MIDDLE;
            }
            else if (p == Position.RIGHT){
                return AutoCommand.B_RIGHT;
            }
        }
        else if (a == Action.ACTION_C){
            if (p == Position.LEFT){
                return AutoCommand.C_LEFT;
            }   
            else if (p == Position.MIDDLE){
                return AutoCommand.C_MIDDLE;
            }
            else if (p == Position.RIGHT){
                return AutoCommand.C_RIGHT;
            }
            */
}    
    
    

