package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import java.util.Optional;
import java.util.OptionalInt;

public class LimeLightSensor extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private static LimeLightSensor instance;

    private LimeLightSensor()
    {
    }

    public static LimeLightSensor getInstance(){
        if(instance == null)
        {
            instance = new LimeLightSensor();
        }
        return instance;
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

    public Pose2d getAmpMovePosition()
    {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        OptionalInt location = DriverStation.getLocation();
        if(alliance.isEmpty() || location.isEmpty())
        {
            return DriveSubsystem.getInstance().getPose2d();
        }
        switch(alliance.get())
        {
            case Red :
                //return new Pose2d();
            case Blue :
                //return new Pose2d();
        }
        return DriveSubsystem.getInstance().getPose2d();
    }

    public void setSpeakerTargetID()
    {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()) {
            return;
        }
        switch(alliance.get())
        {
            case Red -> table.getEntry("priorityid").setInteger(4);
            case Blue -> table.getEntry("priorityid").setInteger(7);
        }
    }

    public void setAmpTargetID()
    {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()) {
            return;
        }
        switch(alliance.get())
        {
            case Red -> table.getEntry("priorityid").setInteger(5);
            case Blue -> table.getEntry("priorityid").setInteger(6);
        }
    }

    public Pose2d getSpeakerTargetPosition()
    {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        OptionalInt location = DriverStation.getLocation();
        if(alliance.isEmpty() || location.isEmpty())
        {
            return DriveSubsystem.getInstance().getPose2d();
        }
        switch(alliance.get())
        {
            case Red :
                switch(location.getAsInt()) {
                    case 1 :
                        //return new Pose2d();
                    case 2 :
                        //return new Pose2d();
                    case 3 :
                        //return new Pose2d();
                }
                break;
            case Blue :
                switch(location.getAsInt()) {
                    case 1 :
                        //return new Pose2d();
                    case 2 :
                        //return new Pose2d();
                    case 3 :
                        //return new Pose2d();
                }
                break;
        }
        return DriveSubsystem.getInstance().getPose2d();
    }

    public double getSpeakerTargetHorizontalOffset()
    {
        return table.getEntry("tx").getDouble(0);
    }
}