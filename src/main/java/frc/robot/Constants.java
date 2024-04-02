// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

    public static boolean DEBUG_INFO = false;
    public static double deadZoneDefault;

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int PAYLOAD_CONTROLLER_PORT = 1;
    }

    public static final class SwerveModuleTest {
        public static final boolean swerveTestMode = false;

        public static final int testModuleIndex = 1;
    }

    public static final class CANIds {
        public static final int PDH_ID = 1;
        public static final SwerveCANId leftFront = new SwerveCANId(4, 5, 22); // module 2
        public static final SwerveCANId rightFront = new SwerveCANId(2, 3, 21); // module 1
        public static final SwerveCANId rightBack = new SwerveCANId(6, 7, 23); // module 3
        public static final SwerveCANId leftBack = new SwerveCANId(8, 9, 24); // module 4

        public static final int intakeDriveMotor = 61;
        public static final int intakeTurnMotor = 60;
        public static final int INTAKE_MOTOR = 11;
        public static final int INTAKE_VERTICAL = 12;

        public static final int SHOOTER_MOTOR_INTAKE = 13;
        public static final int SHOOTER_MOTOR_A = 14;
        public static final int SHOOTER_MOTOR_B = 15;
        public static final int SHOOTER_AMP_EXTENDER = 16;
        public static final int SHOOTER_MOTOR_ELEVATION = 17;
        public static final int CLIMBER_LEFT = 18;
        public static final int CLIMBER_RIGHT = 19;
    }

    public static class DigitalInputs {
        public static final int INTAKE_INSIDE_A = 0;
        public static final int INTAKE_INSIDE_B = 1;
        public static final int INTAKE_IN = 2;
        public static final int INTAKE_OUT = 3;

    }
    public static final class PositionConstants
    {
        public static final double SHOOTER_LAUNCH_AMP = -10.18;
        public static final double MIN_POSE_THRESHOLD = 0.1;
        public static final double MIN_HEADING_THRESHOLD = 5;
        public static final double MIN_POSITION_THRESHOLD = .5;
        public static final double INTAKE_OUT_POSITION = -48.8;
        public static final double INTAKE_IN_POSITION = -0;
        public static final double INTAKE_TRANSFER_POSITION = -0.6;
        public static final double SHOOTER_TRANSFER_POSITION = -13.5;
    }

    public static final class LEDConstants
    {
        public static final int MAX_STRENGTH = 9;
        public static final int MID_STRENGTH = 7;
        public static final int LOW_STRENGTH = 5;
    }

    public static final class EncoderPorts {
        public static final int intakePort = 0;
    }

    public static final class SwerveCANId {
        public final int driveMotor;
        public final int steeringMotor;
        public final int encoder;

        public SwerveCANId(int driveMotor, int steeringMotor, int encoder) {
            this.driveMotor = driveMotor;
            this.steeringMotor = steeringMotor;
            this.encoder = encoder;
        }
    }
}
