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
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SwerveModule<T extends MotorController> {
    private final T driveMotor;
    private final T steeringMotor;
    private final PIDController controller;
    public final CANcoder encoder;
    public RelativeEncoder driveEncoder;
    public final Vector2d position;
    protected double radius; // The distance from the center of the robot to the wheel

    private Direction driveDirection = Direction.FORWARD;
    private Direction turnDirection = Direction.FORWARD;

    private Direction encoderPolarity  = Direction.FORWARD;

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);
        private final int direction;

        Direction(int direction) {
            this.direction = direction;
        }
    }

    public SwerveModule(T driveMotor, T steeringMotor, CANcoder encoder, PIDGains pidGains, Vector2d position, double encoderOffsetAngle) {
        this.driveMotor = driveMotor;
        if (driveMotor instanceof CANSparkMax) {
            driveEncoder = ((CANSparkMax) driveMotor).getEncoder();
            ((CANSparkMax) driveMotor).setSmartCurrentLimit(80, 40);
            ((CANSparkMax) driveMotor).setOpenLoopRampRate(Constants.driveRampRate);
            ((CANSparkMax) steeringMotor).setOpenLoopRampRate(Constants.rotRampRate);
        }
        this.steeringMotor = steeringMotor;
        this.encoder = encoder;
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
        return encoder.getAbsolutePosition().getValue() * 2 * Math.PI;
    }

    public void drive(double speed, double targetAngle) {
        targetAngle = AngleHelpers.unsigned_0_to_2PI(targetAngle);
        double currentAngle = getAngleRadians();

        // err is how many radians the robot is off from its target angle
        double err = AngleHelpers.getError(targetAngle, currentAngle);
        double polarity = 1;
        if (Math.abs(err) > Math.PI / 2) { // Most of the time, the module will drive forward.  However, if the module is more than 90 degrees away from its target angle, it is more efficient for it to drive in reverse towards a target angle offset by 180 degrees from the original.
            err = AngleHelpers.getError((targetAngle + Math.PI) % (2 * Math.PI), currentAngle);
            polarity = -1;
        }

        driveMotor.set(MathUtil.clamp(speed * polarity * driveDirection.direction, -1.0, 1.0));
        steeringMotor.set(MathUtil.clamp(controller.calculate(0, err), -1.0, 1.0) * turnDirection.direction);
    }

    public void rotateAndDrive(Vector2d driveVector, double rotSpeed) {
        double theta = position.angle - driveVector.angle;
        Vector2d velocityVector = new Vector2d(driveVector.magnitude - radius * rotSpeed * Math.sin(theta), radius * rotSpeed * Math.cos(theta));
        drive(velocityVector.magnitude, velocityVector.angle + driveVector.angle - Math.PI / 2);
    }

    public SwerveModulePosition getOdometryData() {
        return new SwerveModulePosition(encoderPolarity.direction * driveEncoder.getPosition() / Constants.neoTicksPerRev / Constants.gearRatio * Constants.wheelDiameter.getValueM() * Math.PI, new Rotation2d(getAngleRadians() + Math.PI / 2));
    }
}
