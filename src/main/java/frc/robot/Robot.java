// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.EnableLogging;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.SmartShuffleboard;
import frc.robot.utils.diag.Diagnostics;
import frc.robot.utils.logging.Logging;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static RobotContainer m_robotContainer;
  private static Diagnostics diagnostics;

  private static boolean isLogging = true;

  private static TARGETING_STATE target_state;

  public enum TARGETING_STATE {OFF, LOCK, CLIMB};
  private UsbCamera USBCam;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    diagnostics = new Diagnostics();
    m_robotContainer = new RobotContainer();
    m_robotContainer.installCommandsOnShuffleboard();
    m_robotContainer.installDriverShuffleboard();
    target_state = TARGETING_STATE.OFF;
    USBCam = CameraServer.startAutomaticCapture(0);
    USBCam.setResolution(320, 240);
    USBCam.setFPS(15);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartShuffleboard.put("Climber", "Can Climb", m_robotContainer.getCanClimb());

    SmartShuffleboard.putCommand("Driver", "Enable Logging", new EnableLogging());

    SmartShuffleboard.put("Driver", "Targeting?", target_state == TARGETING_STATE.LOCK);

    if (isLogging) {
      Logging.instance().writeAllData();
    }

    //IF THE TURRET LIMIT SWITCH IS TRIPPED, YOU CAN CLIMB
    m_robotContainer.setCanClimb(m_robotContainer.getTurretSubsystem().getLeftSwitch());

    if (Constants.ENABLE_DEBUG) {
      SmartShuffleboard.put("Shooter", "State", target_state.name());
    }
    boolean can_shoot = false;
    boolean turret_lock_state = m_robotContainer.getTurretSubsystem().getTurretLockState();
    boolean hood_lock_state = m_robotContainer.getHood().getHoodLockState();
    if (hood_lock_state && turret_lock_state) {
      can_shoot = true;
    }
    else {
      can_shoot = false;
    }

    SmartShuffleboard.put("Driver", "Can Shoot", can_shoot);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    Logging.instance().traceMessage(Logging.MessageLevel.INFORMATION, "-----------DISABLED----------");
  }

  @Override
  public void disabledPeriodic() {
    //Need to implement autonomous commands before this line works
    //SmartShuffleboard.put("Autonomous", "AutoCommandVerify", m_robotContainer.autoChooser.getAutonomousCommand(m_robotContainer.autoChooser.getPosition(), m_robotContainer.autoChooser.getAction());
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    target_state = TARGETING_STATE.OFF;

    // schedule the autonomous command (example)
    Logging.instance().traceMessage(Logging.MessageLevel.INFORMATION, "-----------AUTO INIT----------");
    Logging.instance().writeAllTitles();
    // schedule the autonomous command (example)
    final StringBuilder gameInfo = new StringBuilder();
              gameInfo.append("Match Number=");
		          gameInfo.append(DriverStation.getMatchNumber());
		          gameInfo.append(", Alliance Color=");
		          gameInfo.append(DriverStation.getAlliance().toString());
		          gameInfo.append(", Match Type=");
		          gameInfo.append(DriverStation.getMatchType().toString());
    Logging.instance().traceMessage(Logging.MessageLevel.INFORMATION, gameInfo.toString());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    Logging.instance().traceMessage(Logging.MessageLevel.INFORMATION, "-----------TELEOP INIT----------");
    Logging.instance().writeAllTitles();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    diagnostics.reset();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    diagnostics.refresh();
  }

  public static Diagnostics getDiagnostics() {
    return diagnostics;
  }

  public static RobotContainer getRobotContainer(){
    return m_robotContainer;
  }

  public static void setIsLogging(boolean isLog) {
    isLogging = isLog;
  }

  public static TARGETING_STATE getTargetState() {
    return target_state;
  }

  public static void setTargetState(TARGETING_STATE state){
    target_state = state;
  }
}

