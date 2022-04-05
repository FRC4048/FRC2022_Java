/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.logging.Logging;

/**
 * Add your docs here.
 */
public class MotorUtils {
    public static final double DEFAULT_TIMEOUT = 0.15;
    private double timeout;
    private double time;
    private PowerDistribution powerDistPanel; 

    final int PDPChannel;
    final double currentThreshold;

    private boolean everStalled = false;

    private boolean useEncoder = false;
    private double lastEncoderPosition = 0.0;
    private double encoderThreshold = 10.0;
    private WPI_TalonSRX talon;

    public MotorUtils(int PDPPort, double currentThreshold, double timeout, PowerDistribution powerDistPanel ){
        this.timeout = timeout;
        this.PDPChannel = PDPPort;
        this.currentThreshold = currentThreshold;
        this.powerDistPanel = powerDistPanel;
        time = Timer.getFPGATimestamp();
    }

    /**
     * Initialize a MotorUtils object to monitor for stall conditions
     * @param PDPPort  The PDP port the motor is connected to
     * @param currentThreshold  How many amps the motor will pull before stalling
     * @param timeout  How long before declaring a stall condition
     * @param powerDistPanel The PDP, needed for checking current
     * @param talon  The motor to monitor, must have encoder enabled
     * @param encoderThreshold  The encoder threshold, under which the motor will be stalling
     */
    public MotorUtils(
        int PDPPort, 
        double currentThreshold, 
        double timeout, 
        PowerDistribution powerDistPanel, 
        WPI_TalonSRX talon, 
        double encoderThreshold) {
        this(PDPPort, currentThreshold, timeout, powerDistPanel);
        if (talon == null) {
            throw new NullPointerException("Cannot Initialize with null Talon");
        }
        this.talon = talon;
        lastEncoderPosition = talon.getSelectedSensorPosition();
        useEncoder = true;
        this.encoderThreshold = encoderThreshold;
    }

    public boolean isStalled() {
        if (useEncoder) {
            return checkEncoderAndCurrent();
        }
        final double currentValue = powerDistPanel.getCurrent(PDPChannel);
        final double now = Timer.getFPGATimestamp();

        if (currentValue < currentThreshold) {
            time = now;
        } else {
            DriverStation.reportError("Motor stall, PDP Channel=" + PDPChannel, false);
            if (now - time > timeout) {
                everStalled = true;
                Logging.instance().traceMessage(Logging.MessageLevel.INFORMATION, "Motor stall, PDP channel =" + PDPChannel);
                return true;
            }
        }
        return false;
    }

    public boolean everStalled() {
        return everStalled;
    }

    public void resetStall() {
        everStalled = false;
    }

    private boolean checkEncoderAndCurrent() {
        final double currentValue = powerDistPanel.getCurrent(PDPChannel);
        final double now = Timer.getFPGATimestamp();
        final double currentEncoder = talon.getSelectedSensorPosition();

        final double encoderTravel = (currentEncoder - lastEncoderPosition);

        boolean isStalled = false;

        if (encoderTravel > encoderThreshold) {
            if (currentValue < currentThreshold) {
                // Current is low, motor is turning, no problems
                time = now;
            } else {
                // Is this a concern? Motor is turning but high current
                // If not this all gets collapsed to one if statement
                time = now;
            }
        } else if (encoderTravel < encoderThreshold) {
            if (currentValue < currentThreshold) {
                // Motor is not turning but current is very low, probably not a problem
                time = now;
            } else {
                // Motor is not turning and current is high, bad
                DriverStation.reportError("Motor stall, PDP Channel=" + PDPChannel, false);
                if (now - time > timeout) {
                    Logging.instance().traceMessage(Logging.MessageLevel.INFORMATION, "Motor stall, PDP channel =" + PDPChannel);
                    isStalled = true;    
                    everStalled = true;
                }
            }
        }
        return isStalled;
    }
}