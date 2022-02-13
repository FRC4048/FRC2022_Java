// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utils.logging.Logging;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Add your docs here.
 */
public class PowerDistPanel extends SubsystemBase{
    private PowerDistribution pdp;

    public PowerDistPanel() {
        pdp = new PowerDistribution(Constants.PDP_CAN_ID, ModuleType.kCTRE);
	}

    public Logging.LoggingContext loggingContext = new Logging.LoggingContext(this.getClass()) {

        @Override
        protected void addAll() {

            add("Total Voltage", pdp.getVoltage());
            add("Total Current", pdp.getTotalCurrent());
            add("Drive R1", pdp.getCurrent(Constants.PDP_DRIVE_R1));
            add("Drive R2", pdp.getCurrent(Constants.PDP_DRIVE_R2));
            add("Drive L1", pdp.getCurrent(Constants.PDP_DRIVE_L1));
            add("Drive L2", pdp.getCurrent(Constants.PDP_DRIVE_L2));
        }
    };

    public PowerDistribution getPDP() {
        return pdp;
    }
    public void periodic() {
	}
}
