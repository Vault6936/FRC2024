package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPosition extends Command {
    DriveSubsystem drive;
    Pose2d target;

    PIDController xController = new PIDController(0,0,0);
    PIDController yController = new PIDController(0,0,0);
    PIDController zController = new PIDController(0.00175,0,0);

    public DriveToPosition(DriveSubsystem driveBase, Pose2d targetPosition)
    {
        drive = driveBase;
        target = targetPosition;
    }

    @Override
    public void initialize()
    {
        zController.setSetpoint(target.getRotation().getDegrees());
    }

    @Override
    public void execute()
    {
        SmartDashboard.putNumber("GyroAngle ", drive.getGyroRotation().getDegrees());
        SmartDashboard.putNumber("GyroTarget ", target.getRotation().getDegrees());
        double rotPower = -zController.calculate(drive.getGyroRotation().getDegrees());
        drive.drive(0, 0, rotPower, -1);
    }

    @Override
    public boolean isFinished()
    {
//        if(Math.abs(drive.getPose2d().getX() - target.getX()) > Constants.PositionConstants.MIN_POSE_THRESHOLD)
//        {
//            return false;
//        }
//        if(Math.abs(drive.getPose2d().getY() - target.getY()) > Constants.PositionConstants.MIN_POSE_THRESHOLD)
//        {
//            return false;
//        }
        if(Math.abs(drive.getPose2d().getRotation().getRadians() - target.getRotation().getRadians()) > Constants.PositionConstants.MIN_HEADING_THRESHOLD)
        {
            return false;
        }
        return true;
    }
}
