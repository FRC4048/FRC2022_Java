package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.SmartShuffleboard;


public class TurnDegrees extends CommandBase{
    private final double ANGLE_THRESHOLD = 2;
    private final double MAX_SPEED = 0.5;
    private final double MIN_SPEED = 0.2;   
    private final int SLOWDOWN_ANGLE = 45;
    private final double MAXIMUM_TIME_S = 10;
    private DriveTrain driveTrain;
    private double requiredAngle;
    private double currAngle;
    private double speed = 0.0;
    private double startTime;
    

    public TurnDegrees(DriveTrain driveTrain, int requiredAngle){
        this.driveTrain = driveTrain;
        this.requiredAngle = requiredAngle;
        addRequirements(driveTrain);

    }
    public void initialize(){
      startTime  = Timer.getFPGATimestamp();
    }
    public void execute(){
        currAngle = driveTrain.getAngle();
        double angleError = requiredAngle-currAngle;

        if (Math.abs(angleError) <= ANGLE_THRESHOLD) {
            speed = 0.0;
        } else {
            if (Math.abs(angleError) > SLOWDOWN_ANGLE)
              speed = MAX_SPEED;
            else
              speed = (MAX_SPEED - MIN_SPEED) * (Math.abs(angleError)/SLOWDOWN_ANGLE) + MIN_SPEED;
        }

        if (requiredAngle < currAngle){
          driveTrain.drive(-Math.abs(speed), Math.abs(speed), false);
        } 
        else if (currAngle < requiredAngle){
          driveTrain.drive(Math.abs(speed), -Math.abs(speed), false);
        }

        if (Constants.ENABLE_DEBUG == true){
          SmartShuffleboard.put("Turn", "Right speed", -speed);
          SmartShuffleboard.put("Turn", "Left speed", speed);
          SmartShuffleboard.put("Turn", "Error", angleError);
        }
    }
    
    @Override
    public void end(boolean interrupted){
        driveTrain.drive(0, 0, false);
    }
    @Override
    public boolean isFinished(){
      double elapsedTime = Timer.getFPGATimestamp() - startTime;

      return (Math.abs(currAngle - requiredAngle) <= ANGLE_THRESHOLD) || (elapsedTime >= MAXIMUM_TIME_S);
    }
}
