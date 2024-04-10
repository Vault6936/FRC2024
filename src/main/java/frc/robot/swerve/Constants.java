package frc.robot.swerve;

public final class Constants {

    public static final boolean DEBUG_INFO = false;
    public static final double MAX_VOLTAGE_FOR_MOTORS = 11.0;
    public static final double DRIVE_DEFAULT_SPEED_LIMIT = 0.95;
    public static final double DRIVE_DEFAULT_ACCEL_LIMIT = 0.01;
    public static final double driveMultiplier = 1.0;

    public static final double driveMinSpeed = 0.05;
    public static final double driveRampRate = 100.0;
    public static final double rotMultiplier = 0.3;
    public static final double rotRampRate = 25.0;
    public static final Distance wheelDiameter = new Distance(4.0, Distance.Unit.IN);
    public static final int driveMotorTicksPerRev = 1;
    public static final double gearRatio = 6.75;

    public static final double pathSpeedMultiplier = 0.8;
}