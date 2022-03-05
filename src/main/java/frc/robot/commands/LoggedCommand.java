// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

/** Add your docs here. */
import java.util.Set;
import java.util.TreeSet;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.utils.logging.Logging;
import frc.robot.utils.logging.Logging.MessageLevel;

abstract public class LoggedCommand extends CommandBase {
	private final String ident;
	private final Set<String> requirements = new TreeSet<String>();

	public LoggedCommand() {
		ident = getName();
	}

	private void log(final String text) {
		final StringBuilder sb = new StringBuilder();
		sb.append(this.getClass().getSimpleName());
		sb.append(" ");
		sb.append(ident);
		Logging.instance().traceMessage(MessageLevel.INFORMATION, sb.toString(), requirements.toString(), text);
	}

	@Override
	final public boolean isFinished() {
		final boolean result = loggedIsFinished();
		if (result)
			log(String.format("isFinished()"));
		return result;
	}

	abstract protected boolean loggedIsFinished();

	@Override
	final public void initialize() {
        requirements.add(getRequirements().toString());
		log("initialize()");
		loggedInitialize();
	}

	abstract protected void loggedInitialize();

	@Override
	final public void execute() {
		//log("execute()");
		loggedExecute();
	}

	abstract protected void loggedExecute();

	@Override
	final public void end(boolean interrupted) {
		log("end()");
        if (interrupted) {
            log("interrupted()");
        }
        loggedEnd(interrupted);
	}

	abstract protected void loggedEnd(boolean interrupted);

	@Override
	final public synchronized void cancel() {
        super.cancel();
		if (DriverStation.isEnabled()) {
			log("cancel()");
		}
	}
}
