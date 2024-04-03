package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LimeLightSensor extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public LimeLightSensor()
    {
    }

    @Override
    public void periodic() {
        double[] botpose = table.getEntry("botpose").getDoubleArray(new double[0]);
        if (botpose.length < 5) {
            return;
        }
        double robot_x = botpose[0];
        double robot_y = botpose[1];
        double robot_z = botpose[2];
        double robot_yaw = botpose[5];
        if(Math.abs(robot_x) < 0.01 && Math.abs(robot_y) < 0.01)
        {
            return;
        }
        DriveSubsystem.getInstance().poseEstimator.addVisionMeasurement(new Pose2d(robot_x, robot_y , Rotation2d.fromDegrees(robot_yaw)), Robot.timer.get());
    }
}
