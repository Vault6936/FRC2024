package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.*;
import frc.robot.webdashboard.DashboardLayout;

import static frc.robot.Constants.CANIds;
import static frc.robot.Constants.SwerveModuleTest.swerveTestMode;
import static frc.robot.Constants.SwerveModuleTest.testModuleIndex;
import static frc.robot.GlobalVariables.pose;

public class DriveSubsystem extends SubsystemBase {
    private static DriveSubsystem instance;
    public final AHRS gyro;
    public final SwerveChassis chassis;
    final SwerveModule leftFront;
    final SwerveModule rightFront;
    final SwerveModule leftBack;
    final SwerveModule rightBack;
    SwerveDriveKinematics kinematics;
    SwerveDrivePoseEstimator poseEstimator;
    private String lfPose = "";
    private String rfPose = "";
    private String lbPose = "";
    private String rbPose = "";

    private String lfAngle = "";
    private String rfAngle = "";
    private String lbAngle = "";
    private String rbAngle = "";
    private int counter = 0;

    private DriveSubsystem() {
        double distance = new Distance(12.375, Distance.Unit.IN).getValueM();
        PIDGains swervePIDGains = new PIDGains(0.5, 0.01, 0.005);
        leftFront = new SwerveModule(new CANSparkMax(CANIds.leftFront.driveMotor, CANSparkLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.leftFront.steeringMotor, CANSparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.leftFront.encoder), swervePIDGains, new Vector2d(-distance, distance), -40);
        leftFront.setDriveMotorDirection(SwerveModule.Direction.FORWARD);
        leftFront.setSteeringMotorDirection(SwerveModule.Direction.REVERSE);
        leftFront.setEncoderPolarity(SwerveModule.Direction.FORWARD);
        rightFront = new SwerveModule(new CANSparkMax(CANIds.rightFront.driveMotor, CANSparkLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.rightFront.steeringMotor, CANSparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.rightFront.encoder), swervePIDGains, new Vector2d(distance, distance), 24);
        rightFront.setDriveMotorDirection(SwerveModule.Direction.REVERSE);
        rightFront.setSteeringMotorDirection(SwerveModule.Direction.REVERSE);
        rightFront.setEncoderPolarity(SwerveModule.Direction.REVERSE);
        leftBack = new SwerveModule(new CANSparkMax(CANIds.leftBack.driveMotor, CANSparkLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.leftBack.steeringMotor, CANSparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.leftBack.encoder), swervePIDGains, new Vector2d(-distance, -distance), -35);
        leftBack.setDriveMotorDirection(SwerveModule.Direction.FORWARD);
        leftBack.setSteeringMotorDirection(SwerveModule.Direction.REVERSE);
        leftBack.setEncoderPolarity(SwerveModule.Direction.FORWARD);
        rightBack = new SwerveModule(new CANSparkMax(CANIds.rightBack.driveMotor, CANSparkLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.rightBack.steeringMotor, CANSparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.rightBack.encoder), swervePIDGains, new Vector2d(distance, -distance), -34);
        rightBack.setDriveMotorDirection(SwerveModule.Direction.REVERSE);
        rightBack.setSteeringMotorDirection(SwerveModule.Direction.REVERSE);
        rightBack.setEncoderPolarity(SwerveModule.Direction.REVERSE);
        chassis = new SwerveChassis(leftFront, rightFront, leftBack, rightBack);
        chassis.setDriveLimit(SwerveChassis.DriveLimits.DEFAULT);
        chassis.setRotationLimit(SwerveChassis.DriveLimits.NONE);
        gyro = new AHRS();

        kinematics = new SwerveDriveKinematics(leftFront.position.toTranslation2d(), rightFront.position.toTranslation2d(), leftBack.position.toTranslation2d(), rightBack.position.toTranslation2d());
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroRotation(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));
    }

    public static DriveSubsystem getInstance() {
        if (instance == null) instance = new DriveSubsystem();
        return instance;
    }

    public void boot() {
        lfPose = "";
        rfPose = "";
        lbPose = "";
        rbPose = "";
    }

    public void drive(double x, double y, double rot, double speedM) {
        speedM = (speedM * -0.5) + 0.5;
        if (swerveTestMode) {
            Vector2d vector = new Vector2d(x, y);
            ((SwerveModule) chassis.modules.get(testModuleIndex)).drive(vector.magnitude * speedM, vector.angle, false);
        } else {
            chassis.drive(x * speedM, -y * speedM, -rot * speedM);
        }
    }

    public void resetGyro() {
        resetGyro(0);
    }

    public void resetGyro(double angle)
    {
        gyro.resetDisplacement();
        gyro.zeroYaw();
        gyro.setAngleAdjustment(angle);
        pose = new Pose2d(0, 0, new Rotation2d(angle));
        chassis.resetPose();
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{leftFront.getOdometryData(), rightFront.getOdometryData(), leftBack.getOdometryData(), rightBack.getOdometryData()};
    }

    public Rotation2d getGyroRotation() {
        return new Rotation2d(Math.toRadians(-gyro.getAngle()));
    }

    public Pose2d getPose2d() {
        return pose;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            return;
        }
        counter++;

        if(false) {
            DashboardLayout.setNodeValue("encoder2", rightFront.getAngleRadians() * 180 / Math.PI);
            DashboardLayout.setNodeValue("encoder3", leftBack.getAngleRadians() * 180 / Math.PI);
            DashboardLayout.setNodeValue("encoder4", rightBack.getAngleRadians() * 180 / Math.PI);
            DashboardLayout.setNodeValue("bot angle", pose.getRotation().getDegrees());
            DashboardLayout.setNodeValue("pose", "x:" + pose.getTranslation().getX() + "y:" + pose.getTranslation().getY() + "heading:" + pose.getRotation().getRadians());

            lfPose += counter + ", " + leftFront.getOdometryData().distanceMeters + ", ";
            rfPose += counter + ", " + rightFront.getOdometryData().distanceMeters + ", ";
            lbPose += counter + ", " + leftBack.getOdometryData().distanceMeters + ", ";
            rbPose += counter + ", " + rightBack.getOdometryData().distanceMeters + ", ";

            lfAngle += counter + ", " + leftFront.getAngleRadians() + ", ";
            rfAngle += counter + ", " + rightFront.getAngleRadians() + ", ";
            lbAngle += counter + ", " + leftBack.getAngleRadians() + ", ";
            rbAngle += counter + ", " + rightBack.getAngleRadians() + ", ";

            DashboardLayout.setNodeValue("lf pose", lfPose);
            DashboardLayout.setNodeValue("rf pose", rfPose);
            DashboardLayout.setNodeValue("lb pose", lbPose);
            DashboardLayout.setNodeValue("rb pose", rbPose);
            SmartDashboard.putString("lf pose", lfPose);
            SmartDashboard.putString("rf pose", rfPose);
            SmartDashboard.putString("lb pose", lbPose);
            SmartDashboard.putString("rb pose", rbPose);
        }

        pose = poseEstimator.update(getGyroRotation(), getModulePositions());
        SmartDashboard.putString("RobotPoseX", String.format("%.4f", pose.getX()));
        SmartDashboard.putString("RobotPoseY", String.format("%.4f", pose.getY()));
        SmartDashboard.putString("RobotPoseO", String.format("%.4f", pose.getRotation().getDegrees()));

    }
}
