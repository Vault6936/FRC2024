package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPosition extends Command {
    DriveSubsystem drive = DriveSubsystem.getInstance();
    Pose2d target;
    boolean hasGotToPosition = false;
    double timeGotToPosition = -1;

    PIDController xController = new PIDController(7,0,0);
    PIDController yController = new PIDController(7,0,0);
    PIDController zController = new PIDController(0.05,0,0);

    public DriveToPosition(Pose2d targetPosition)
    {
        target = targetPosition;
        addRequirements(drive);
    }

    @Override
    public void initialize()
    {
        hasGotToPosition = false;
        zController.setSetpoint(target.getRotation().getDegrees());
        yController.setSetpoint(target.getY());
        xController.setSetpoint(target.getX());
    }

    @Override
    public void execute()
    {
        SmartDashboard.putNumber("GyroAngle ", drive.getGyroRotation().getDegrees());
        SmartDashboard.putNumber("GyroTarget ", target.getRotation().getDegrees());
        double rotPower = MathUtil.clamp(-zController.calculate(drive.getGyroRotation().getDegrees()),
                -0.75, 0.75);

        SmartDashboard.putNumber("TargetY ", target.getY());
        double YPower = MathUtil.clamp(-yController.calculate(drive.getPose2d().getY()),
                -0.75, 0.75);

        SmartDashboard.putNumber("TargetX ", target.getX());
        double XPower = MathUtil.clamp(-xController.calculate(drive.getPose2d().getX()),
                -0.75, 0.75);
        drive.drive(-XPower*Math.sin((Math.PI)/2), YPower*Math.sin((Math.PI)/2), rotPower*Math.sin((Math.PI)/2), -1);
    }

    @Override
    public boolean isFinished()
    {
        if(Math.abs(drive.getPose2d().getX() - target.getX()) > Constants.PositionConstants.MIN_POSE_THRESHOLD)
        {
            return false;
        }
        if(Math.abs(drive.getPose2d().getY() - target.getY()) > Constants.PositionConstants.MIN_POSE_THRESHOLD)
        {
            return false;
        }
        if(Math.abs(drive.getPose2d().getRotation().getDegrees() - target.getRotation().getDegrees()) > Constants.PositionConstants.MIN_HEADING_THRESHOLD)
        {
            return false;
        }
        if(!hasGotToPosition)
        {
            hasGotToPosition = true;
            timeGotToPosition = Robot.timer.get();
        }
        if(Math.abs(Robot.timer.get() - timeGotToPosition) < 0.5) {
            return false;
        }
        return true;
    }
}
