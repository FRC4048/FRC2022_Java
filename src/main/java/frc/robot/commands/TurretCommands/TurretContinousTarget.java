// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Robot.TARGETING_STATE;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.limelight.LimeLightVision;

public class TurretContinousTarget extends CommandBase {
  /** Creates a new TurretAutoCommand. */
  private TurretSubsystem turret;
  private LimeLightVision limelight;
  private DoubleSupplier joystick;
  private double turretSpeed;
  private double speed;

  public TurretContinousTarget(TurretSubsystem turret, LimeLightVision limelight, DoubleSupplier joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.limelight = limelight;
    this.joystick = joystick;
    turretSpeed = Constants.TURRET_CLOCKWISE_SPEED;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // starting state?
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(Robot.getTargetState()) {
      case OFF:
        turret.setTurret((joystick.getAsDouble() * Constants.TURRETSPIN_SCALEFACTOR));
        break;

      case LOCK:
        if (limelight.hasTarget()) {
          double tx = limelight.getCameraAngles().getTx();
            if (tx > (4 + Constants.TURRET_ERROR_THRESHOLD) || tx > (4 - Constants.TURRET_ERROR_THRESHOLD)) {
                speed = Constants.TURRET_SLOW_SPEED; 
            }
            else {
                speed = Constants.TURRET_SLOW_SPEED;
            }
            if (tx > 4) {
                turret.setTurret(speed * -1);
            } else if (tx < 4) {
                turret.setTurret(speed);
            }
        } else {  
            if (((turret.getEncoder() >= Constants.TURRET_RIGHT_THRESHOLD) && turretSpeed < 0) || 
                ((turret.getEncoder() <= Constants.TURRET_LEFT_THRESHOLD) && turretSpeed > 0)) {
                  turretSpeed = -turretSpeed;
            }
            turret.setTurret(turretSpeed);
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setTurret(0);
    Robot.setTargetState(TARGETING_STATE.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
