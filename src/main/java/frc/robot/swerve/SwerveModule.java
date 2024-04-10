package frc.robot.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.webdashboard.WebdashboardServer;

import static frc.robot.swerve.Constants.gearRatio;
import static frc.robot.swerve.Constants.wheelDiameter;

public class SwerveModule {
    public final CANcoder angleEncoder;
    public final Vector2d position;
    private final CANSparkMax driveMotor;
    private final CANSparkMax steeringMotor;
    private final PIDController controller;
    public RelativeEncoder driveEncoder;
    protected double radius; // The distance from the center of the robot to the wheel

    private Direction driveDirection = Direction.FORWARD;
    private Direction turnDirection = Direction.FORWARD;

    private Direction encoderPolarity = Direction.FORWARD;

    private double lastStoredAngle = 0;

    public SwerveModule(CANSparkMax driveMotor, CANSparkMax steeringMotor, CANcoder encoder, PIDGains pidGains, Vector2d position, double encoderOffsetAngle) {
        this.driveMotor = driveMotor;
        driveEncoder = driveMotor.getEncoder();
        driveMotor.setSmartCurrentLimit(80, 40);
        driveMotor.setOpenLoopRampRate(Constants.driveRampRate);
        steeringMotor.setOpenLoopRampRate(Constants.rotRampRate);
        this.steeringMotor = steeringMotor;
        this.angleEncoder = encoder;
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.MagnetOffset = encoderOffsetAngle / 360;
        encoder.getConfigurator().apply(magnetSensorConfigs);
        this.position = position;
        controller = new PIDController(pidGains.kP, pidGains.kI, pidGains.kD);
        boot();
    }

    public void boot() {
        driveEncoder.setPosition(0);
        for (int i = 0; i < 30; i++) {
            if(Constants.DEBUG_INFO) {
                System.out.println("booted");
            }
        }
    }

    public void setDriveMotorDirection(Direction direction) {
        driveDirection = direction;
    }

    public void setSteeringMotorDirection(Direction direction) {
        turnDirection = direction;
    }

    public void setEncoderPolarity(Direction direction) {
        encoderPolarity = direction;
    }

    public double getAngleRadians() {
        return angleEncoder.getAbsolutePosition().getValue() * 2 * Math.PI;
    }

    public Rotation2d getRotation2d() {return new Rotation2d(getAngleRadians());}

    public double getSpeedM_s() { return driveEncoder.getVelocity() * Math.PI * wheelDiameter.getValueM() / gearRatio / 60;}

    public SwerveModuleState getState() { return new SwerveModuleState(getSpeedM_s(),getRotation2d());}

    public void drive(double speed, double targetAngle, boolean inDeadZone) {
        try {
            controller.setD(Double.parseDouble(WebdashboardServer.getInstance(5800).getFirstConnectedLayout().getInputValue("heading_kd")));
        }
        catch (Exception e) {
        }

        targetAngle = AngleHelpers.unsigned_0_to_2PI(targetAngle);
        double currentAngle = getAngleRadians();

        // err is how many radians the robot is off from its target angle
        double err = AngleHelpers.getError(targetAngle, currentAngle);
        double polarity = 1;
        /*if (Math.abs(err) > Math.PI / 2) { // Most of the time, the module will drive forward.  However, if the module is more than 90 degrees away from its target angle, it is more efficient for it to drive in reverse towards a target angle offset by 180 degrees from the original.
            err = AngleHelpers.getError((targetAngle + Math.PI) % (2 * Math.PI), currentAngle);
            polarity = -1;
        }*/
        if(!inDeadZone)
        {
            driveMotor.set(MathUtil.clamp(speed * polarity * driveDirection.direction, -1.0, 1.0));
            //driveMotor.setVoltage(MathUtil.clamp(speed * polarity * driveDirection.direction * Constants.MAX_VOLTAGE_FOR_MOTORS,
            //        -Constants.MAX_VOLTAGE_FOR_MOTORS, Constants.MAX_VOLTAGE_FOR_MOTORS));
            steeringMotor.set(MathUtil.clamp(controller.calculate(0, err), -1.0, 1.0) * turnDirection.direction);
        }
        else
        {
            driveMotor.set(0);
            steeringMotor.set(0);
        }
    }

    public void rotateAndDrive(Vector2d driveVector, double rotSpeed, boolean inDeadZone) {
        double theta = position.angle - driveVector.angle;
        Vector2d velocityVector = new Vector2d(
                driveVector.magnitude - radius * rotSpeed * Math.sin(theta),
                radius * rotSpeed * Math.cos(theta));
        drive(velocityVector.magnitude, velocityVector.angle + driveVector.angle - Math.PI / 2, inDeadZone);
    }

    public SwerveModulePosition getOdometryData() {
        return new SwerveModulePosition(encoderPolarity.direction * driveEncoder.getPosition() /
                Constants.driveMotorTicksPerRev / Constants.gearRatio * wheelDiameter.getValueM() * Math.PI,
                new Rotation2d(getAngleRadians() + Math.PI / 2));
    }

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);
        private final int direction;

        Direction(int direction) {
            this.direction = direction;
        }
    }
}
