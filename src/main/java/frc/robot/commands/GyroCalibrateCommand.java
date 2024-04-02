package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.swerve.AngleHelpers;

public class GyroCalibrateCommand extends Command {

    private final DriveSubsystem subsystem = DriveSubsystem.getInstance();
    private double startTimestamp;
    private final double waitTime;

    public GyroCalibrateCommand(int waitTime) {
        this.waitTime = waitTime;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        startTimestamp = Robot.timer.get() * 1000;
        subsystem.resetGyro();
    }

    @Override
    public void execute() {
        if (Math.abs(AngleHelpers.unsigned_negative180_to_180(GlobalVariables.pose.getRotation().getDegrees())) > 2.0) {
            subsystem.resetGyro();
        }
    }

    @Override
    public boolean isFinished() {
        return !subsystem.gyro.isCalibrating() && Robot.timer.get() * 1000 > startTimestamp + waitTime;
    }
}
