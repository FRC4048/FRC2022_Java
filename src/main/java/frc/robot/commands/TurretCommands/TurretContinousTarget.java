// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.TARGETING_STATE;
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
    switch(turret.getTargetState()) {
      case OFF:
        turret.setTurret((joystick.getAsDouble() * Constants.TURRETSPIN_SCALEFACTOR));
        break;

      case SWEEP:
        if (((turret.getEncoder() >= Constants.TURRET_RIGHT_THRESHOLD) && turretSpeed < 0) || 
            ((turret.getEncoder() <= Constants.TURRET_LEFT_THRESHOLD) && turretSpeed > 0)) {
            turretSpeed = -turretSpeed;
        }
        turret.setTurret(turretSpeed);
        break;

      case LOCK:
        if (limelight.hasTarget()) {
          if (Math.abs(limelight.getCameraAngles().getTx()) > Constants.TURRET_ERROR_THRESHOLD) {
            speed = Constants.TURRET_FAST_SPEED; 
          } else {
            speed = Constants.TURRET_SLOW_SPEED;
          }
          turret.setTurret(-1 * Math.signum(limelight.getCameraAngles().getTx()) * speed);
        } else {  
          turret.setTurret(0);
        }
        break;
    }

    if (limelight.hasTarget()) {
      turret.setTargetState(TARGETING_STATE.LOCK);
    } else {
      turret.setTargetState(TARGETING_STATE.SWEEP);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setTurret(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
