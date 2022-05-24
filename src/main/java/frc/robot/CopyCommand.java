package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.SmartShuffleboard;

public class CopyCommand extends CommandBase {
    @Override
    public void execute() {
        double in = SmartShuffleboard.getDouble("test", "Input", 0.0);
        SmartShuffleboard.put("test", "Output", in);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
