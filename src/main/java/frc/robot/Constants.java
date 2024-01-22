// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.swerve.Distance;

public final class Constants {

    public static double deadZoneDefault;

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
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

        public static final int intakeDriveMotor = 10;
        public static final int intakeTurnMotor = 11;
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
