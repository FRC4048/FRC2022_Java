// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashSet;
/** Add your docs here. */
import java.util.Set;
import java.util.TreeSet;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.logging.Logging;
import frc.robot.utils.logging.Logging.MessageLevel;

abstract public class LoggedCommandBase extends CommandBase {

	private void log(final String text) {
		final StringBuilder sb = new StringBuilder();
		sb.append(this.getClass().getSimpleName());
		sb.append(" ");
		sb.append(getName());
		Logging.instance().traceMessage(MessageLevel.INFORMATION, sb.toString(), m_requirements.toString(), text);
	}

	public void addLog(String text) {
		log(text);
	}

	public void addLog(double value) {
		log(Double.toString(value));
	}

	public void addLog(int value) {
		log(Integer.toString(value));
	}
}
