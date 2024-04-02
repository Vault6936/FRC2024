package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToRelativePositionCommand extends DriveToPosition {
    public DriveToRelativePositionCommand(DriveSubsystem driveBase, Pose2d targetPosition) {
        super(driveBase, targetPosition);
    }

    @Override
    public void initialize()
    {
        Pose2d actualTarget = drive.getPose2d().plus(new Transform2d(target.getX(), target.getY(), target.getRotation()));
        zController.setSetpoint(actualTarget.getRotation().getDegrees());
        yController.setSetpoint(actualTarget.getY());
        xController.setSetpoint(actualTarget.getX());
        target = actualTarget;
    }
}
