// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Robot.TARGETING_STATE;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.limelight.LimeLightVision;

public class  TurretContinousTarget extends CommandBase {
  /** Creates a new TurretAutoCommand. */
  private TurretSubsystem turret;
  private LimeLightVision limelight;
  private DoubleSupplier joystick;
  private double turretSpeed;

  private static final double TARGET_ANGLE = 4;

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
    boolean turretState = false;
    SmartShuffleboard.put("Continous", "Encoder", turret.getEncoder());
    turret.setTurretLockState(false);
    switch (Robot.getTargetState()) {
      case OFF:
        turret.setTurret((joystick.getAsDouble() * Constants.TURRETSPIN_SCALEFACTOR));
        break;
      
      case CLIMB:
        turret.setTurret(0.15);
        break;

      case LOCK:
        double speed;
        if (limelight.hasTarget()) {
          double tx = limelight.getCameraAngles().getTx();
          turret.setTurret(turret.getPID().calculate(tx, 4));
          /*if (Math.abs(tx - TARGET_ANGLE) > Constants.TURRET_ERROR_THRESHOLD) {
            speed = Constants.TURRET_FAST_SPEED;
          } else {
            speed = Constants.TURRET_SLOW_SPEED * (Math.abs(tx - TARGET_ANGLE) / Constants.TURRET_ERROR_THRESHOLD);
          }
          if (Math.abs(TARGET_ANGLE - tx) > .5) {
            turret.setTurret(speed * Math.signum(TARGET_ANGLE - tx));
            
          } else {
            turretState = true;
            turret.setTurret(0);
          } */
          
          
          
        } else {
          if ((((turret.getEncoder() >= Constants.TURRET_RIGHT_THRESHOLD) || turret.getRightSwitch()) && turretSpeed < 0) ||
              (((turret.getEncoder() <= Constants.TURRET_LEFT_THRESHOLD) || turret.getLeftSwitch()) && turretSpeed > 0)) {
            turretSpeed = -turretSpeed;
          }
          SmartShuffleboard.put("Continous", "Speed", turretSpeed);
          turret.setTurret(turretSpeed);
        }
        break;
    }
    turret.setTurretLockState(turretState);
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
