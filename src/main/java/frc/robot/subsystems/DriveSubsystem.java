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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.*;
import frc.robot.webdashboard.DashboardLayout;
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
    public final SwerveChassis<CANSparkMax> chassis;

    SwerveDriveKinematics kinematics;

    SwerveDrivePoseEstimator poseEstimator;

    private static DriveSubsystem instance;

    private DriveSubsystem() {
        double distance = new Distance(12.375, Distance.Unit.IN).getValueM();
        PIDGains swervePIDGains = new PIDGains(0.5, 0.01, 0.001);
        leftFront = new SwerveModule<>(new CANSparkMax(CANIds.leftFront.driveMotor, CANSparkLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.leftFront.steeringMotor, CANSparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.leftFront.encoder), swervePIDGains, new Vector2d(-distance, distance), -40);
        leftFront.setDriveMotorDirection(SwerveModule.Direction.FORWARD);
        leftFront.setSteeringMotorDirection(SwerveModule.Direction.REVERSE);
        leftFront.setEncoderPolarity(SwerveModule.Direction.REVERSE);
        rightFront = new SwerveModule<>(new CANSparkMax(CANIds.rightFront.driveMotor, CANSparkLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.rightFront.steeringMotor, CANSparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.rightFront.encoder), swervePIDGains, new Vector2d(distance, distance), 24);
        rightFront.setDriveMotorDirection(SwerveModule.Direction.REVERSE);
        rightFront.setSteeringMotorDirection(SwerveModule.Direction.REVERSE);
        rightFront.setEncoderPolarity(SwerveModule.Direction.REVERSE);
        leftBack = new SwerveModule<>(new CANSparkMax(CANIds.leftBack.driveMotor, CANSparkLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.leftBack.steeringMotor, CANSparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.leftBack.encoder), swervePIDGains, new Vector2d(-distance, -distance), -35);
        leftBack.setDriveMotorDirection(SwerveModule.Direction.FORWARD);
        leftBack.setSteeringMotorDirection(SwerveModule.Direction.REVERSE);
        leftBack.setEncoderPolarity(SwerveModule.Direction.FORWARD);
        rightBack = new SwerveModule<>(new CANSparkMax(CANIds.rightBack.driveMotor, CANSparkLowLevel.MotorType.kBrushless), new CANSparkMax(CANIds.rightBack.steeringMotor, CANSparkLowLevel.MotorType.kBrushless), new CANcoder(CANIds.rightBack.encoder), swervePIDGains, new Vector2d(distance, -distance), -34);
        rightBack.setDriveMotorDirection(SwerveModule.Direction.REVERSE);
        rightBack.setSteeringMotorDirection(SwerveModule.Direction.REVERSE);
        rightBack.setEncoderPolarity(SwerveModule.Direction.FORWARD);
        chassis = new SwerveChassis<>(leftFront, rightFront, leftBack, rightBack);
        chassis.setDriveLimit(SwerveChassis.DriveLimits.NONE);
        chassis.setRotationLimit(SwerveChassis.DriveLimits.NONE);
        gyro = new AHRS();

        kinematics = new SwerveDriveKinematics(leftFront.position.toTranslation2d(), rightFront.position.toTranslation2d(), leftBack.position.toTranslation2d(), rightBack.position.toTranslation2d());
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroRotation(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));
    }

    public void drive(double x, double y, double rot) {
        if (swerveTestMode) {
            Vector2d vector = new Vector2d(x, y);
            chassis.modules.get(testModuleIndex).drive(vector.magnitude, vector.angle);
        } else {

            chassis.drive(x, -y, -rot);
        }
    }

    public void resetGyro() {
        gyro.resetDisplacement();
        gyro.zeroYaw();
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{leftFront.getOdometryData(), rightFront.getOdometryData(), leftBack.getOdometryData(), rightBack.getOdometryData()};
    }

    public Rotation2d getGyroRotation() {
        return new Rotation2d(Math.toRadians(-gyro.getAngle()));
    }
    @Override
    public void periodic() {
        DashboardLayout.setNodeValue("lf pose", leftFront.driveEncoder.getPosition());
        DashboardLayout.setNodeValue("rf pose", rightFront.driveEncoder.getPosition());
        DashboardLayout.setNodeValue("lb pose", leftBack.driveEncoder.getPosition());
        DashboardLayout.setNodeValue("rb pose", rightBack.driveEncoder.getPosition());

        DashboardLayout.setNodeValue("encoder1", leftFront.getAngleRadians() * 180 / Math.PI);
        DashboardLayout.setNodeValue("encoder2", rightFront.getAngleRadians() * 180 / Math.PI);
        DashboardLayout.setNodeValue("encoder3", leftBack.getAngleRadians() * 180 / Math.PI);
        DashboardLayout.setNodeValue("encoder4", rightBack.getAngleRadians() * 180 / Math.PI);
        DashboardLayout.setNodeValue("bot angle", pose.getRotation().getDegrees());
        DashboardLayout.setNodeValue("pose", "x:" + pose.getTranslation().getX() + "y:" + pose.getTranslation().getY() + "heading:" + pose.getRotation().getRadians());

        pose = poseEstimator.update(getGyroRotation(), getModulePositions());
    }

    public static DriveSubsystem getInstance() {
        if (instance == null) instance = new DriveSubsystem();
        return instance;
    }
}
