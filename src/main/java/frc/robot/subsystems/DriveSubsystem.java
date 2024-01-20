package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.*;
import frc.robot.webdashboard.DashboardLayout;
import java.util.ArrayList;
import static frc.robot.Constants.CANIds;
import static frc.robot.Constants.SwerveModuleTest.swerveTestMode;
import static frc.robot.Constants.SwerveModuleTest.testModuleIndex;
import static frc.robot.GlobalVariables.pose;

public class DriveSubsystem extends SubsystemBase {
    public final AHRS gyro;
    final SwerveModule<CANSparkMax> leftFront;
    final SwerveModule<CANSparkMax> rightFront;
    final SwerveModule<CANSparkMax> leftBack;
    final SwerveModule<CANSparkMax> rightBack;
    private final SwerveChassis<CANSparkMax> chassis;

    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;

    private static DriveSubsystem instance;

    private DriveSubsystem() {
        double distance = new Distance(12.375, Distance.Unit.IN).getValueM();
        PIDGains swervePIDGains = new PIDGains(0.5, 0.01, 0.001);
        leftFront = new SwerveModule<>(new CANSparkMax(CANIds.leftFront.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.leftFront.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANcoder(CANIds.leftFront.encoder), swervePIDGains, new Vector2d(-distance, distance), -40);
        leftFront.setSteeringMotorDirection(SwerveModule.Direction.REVERSE);
        rightFront = new SwerveModule<>(new CANSparkMax(CANIds.rightFront.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.rightFront.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANcoder(CANIds.rightFront.encoder), swervePIDGains, new Vector2d(distance, distance), 24);
        rightFront.setSteeringMotorDirection(SwerveModule.Direction.REVERSE);
        rightFront.setDriveMotorDirection(SwerveModule.Direction.REVERSE);
        leftBack = new SwerveModule<>(new CANSparkMax(CANIds.leftBack.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.leftBack.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANcoder(CANIds.leftBack.encoder), swervePIDGains, new Vector2d(-distance, -distance), -35);
        leftBack.setSteeringMotorDirection(SwerveModule.Direction.REVERSE);
        rightBack = new SwerveModule<>(new CANSparkMax(CANIds.rightBack.driveMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.rightBack.steeringMotor, CANSparkMaxLowLevel.MotorType.kBrushless), new CANcoder(CANIds.rightBack.encoder), swervePIDGains, new Vector2d(distance, -distance), -34);
        rightBack.setSteeringMotorDirection(SwerveModule.Direction.REVERSE);
        rightBack.setDriveMotorDirection(SwerveModule.Direction.REVERSE);
        chassis = new SwerveChassis<>(leftFront, rightFront, leftBack, rightBack);
        chassis.setDriveLimit(SwerveChassis.DriveLimits.NONE);
        chassis.setRotationLimit(SwerveChassis.DriveLimits.NONE);
        gyro = new AHRS();

        kinematics = new SwerveDriveKinematics(leftFront.position.toTranslation2d(), rightFront.position.toTranslation2d(), leftBack.position.toTranslation2d(), rightBack.position.toTranslation2d());
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(Math.toRadians(gyro.getAngle())), new SwerveModulePosition[]{leftFront.getOdometryData(), rightFront.getOdometryData(), leftBack.getOdometryData(), rightBack.getOdometryData()}, new Pose2d(0, 0, new Rotation2d(0)));
    }

    public ArrayList<SwerveModule<CANSparkMax>> getModules() {
        return chassis.modules;
    }

    public void drive(double x, double y, double rot) {

        if (swerveTestMode) {
            Vector2d vector = new Vector2d(x, y);
            chassis.modules.get(testModuleIndex).drive(vector.magnitude, vector.angle);
        } else {
            chassis.drive(x, -y, -rot);
        }
    }

    public void calibrateGyro() {
        gyro.calibrate();
        gyro.resetDisplacement();
        gyro.zeroYaw();
    }

    @Override
    public void periodic() {
        pose = odometry.update(new Rotation2d(Math.toRadians(-gyro.getAngle())), new SwerveModulePosition[]{leftFront.getOdometryData(), rightFront.getOdometryData(), leftBack.getOdometryData(), rightBack.getOdometryData()});

        DashboardLayout.setNodeValue("lf pose", leftFront.getOdometryData());
        DashboardLayout.setNodeValue("rf pose", rightFront.getOdometryData());
        DashboardLayout.setNodeValue("lb pose", leftBack.getOdometryData());
        DashboardLayout.setNodeValue("rb pose", rightBack.getOdometryData());

        DashboardLayout.setNodeValue("encoder1", leftFront.getAngleRadians() * 180 / Math.PI);
        DashboardLayout.setNodeValue("encoder2", rightFront.getAngleRadians() * 180 / Math.PI);
        DashboardLayout.setNodeValue("encoder3", leftBack.getAngleRadians() * 180 / Math.PI);
        DashboardLayout.setNodeValue("encoder4", rightBack.getAngleRadians() * 180 / Math.PI);
        DashboardLayout.setNodeValue("bot angle", pose.getRotation().getDegrees());
        DashboardLayout.setNodeValue("pose", "x:" + pose.getTranslation().getX() + "y:" + pose.getTranslation().getY() + "heading:" + pose.getRotation().getRadians());
    }

    public static DriveSubsystem getInstance() {
        if (instance == null) instance = new DriveSubsystem();
        return instance;
    }
}
