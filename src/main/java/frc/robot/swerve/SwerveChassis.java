package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.webdashboard.DashboardLayout;
import frc.robot.webdashboard.WebdashboardServer;

import java.util.ArrayList;


import static frc.robot.GlobalVariables.pose;

public class SwerveChassis<T extends MotorController> {
    public final ArrayList<SwerveModule> modules;
    SwerveModule leftFront;
    SwerveModule rightFront;
    SwerveModule leftBack;
    SwerveModule rightBack;
    PIDController headingController;
    private double targetAngle = 0.0;
    private InputLimit driveLimit = DriveLimits.NONE;
    private InputLimit rotationLimit = DriveLimits.NONE;
    private DriveInput lastInput = new DriveInput(new Vector2d(), 0);

    public SwerveChassis(SwerveModule leftFront, SwerveModule rightFront, SwerveModule leftBack, SwerveModule rightBack) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        modules = new ArrayList<>();
        modules.add(leftFront);
        modules.add(rightFront);
        modules.add(leftBack);
        modules.add(rightBack);

        // Normalize the radii
        double largest = 0.0;
        for (SwerveModule module : modules) {
            if (module.position.magnitude > largest) {
                largest = module.position.magnitude;
            }
        }
        for (SwerveModule module : modules) {
            module.radius = module.position.magnitude / largest;
        }

        headingController = new PIDController(1, 0, 0);
    }

    private static double getCircularDivisor(double x, double y) {
        if (x == 0.0 && y == 0.0) {
            return 1.0;
        } else if (Math.abs(y / x) >= 1.0) {
            return Math.sqrt(1 + Math.pow(x / y, 2));
        } else {
            return Math.sqrt(1 + Math.pow(y / x, 2));
        }
    }

    public void boot() {
        for (SwerveModule module : modules) {
            module.boot();
        }
        targetAngle = 0.0;
    }

    public void setDriveLimit(InputLimit inputLimit) {
        this.driveLimit = inputLimit;
    }

    public void setRotationLimit(InputLimit rotationLimit) {
        this.rotationLimit = rotationLimit;
    }

    public void resetPose()
    {
        for(SwerveModule module : modules)
        {
            module.driveEncoder.setPosition(0);
        }
    }

    public void drive(double x, double y, double rot, boolean squareInputs) {
        boolean inDeadZone = false;
        if (Math.abs(x) < Constants.driveMinSpeed && Math.abs(x) < Constants.driveMinSpeed && Math.abs(rot) < Constants.driveMinSpeed)
        {
            inDeadZone = true;
        }
        if (squareInputs) {
            x = Math.copySign(Math.pow(x, 2), x);
            y = Math.copySign(Math.pow(y, 2), y);
            rot = Math.copySign(Math.pow(rot, 2), rot) * -1;
        }
        double circularDivisor = getCircularDivisor(x, y); // In the joystick API, x and y can be 1 simultaneously - the inputs are bounded by a square, not a circle, so the radius can be as high as 1.41.  This method converts the inputs to circular bounds.
        x /= circularDivisor;
        y /= circularDivisor;
        x *= Constants.driveMultiplier;
        y *= Constants.driveMultiplier;
        rot *= Constants.rotMultiplier;
        Vector2d inputVector = new Vector2d(x, y);

        double divisor = Math.abs(inputVector.magnitude) + Math.abs(rot);
        if (divisor > 1.0) {
            inputVector = new Vector2d(inputVector.magnitude / divisor, inputVector.angle, false);
            rot /= divisor;
        }

        double currentAngle = AngleHelpers.unsigned_0_to_2PI(pose.getRotation().getRadians());

        if (Math.abs(rot) < 0.05 && false) {
            if (Math.abs(lastInput.rot) >= 0.05) {
                targetAngle = currentAngle;
            }
            rot = -MathUtil.clamp(headingController.calculate(0, AngleHelpers.getError(targetAngle, currentAngle)), -0.5, 0.5);
        }

        DashboardLayout.setNodeValue("bot target heading", Math.toDegrees(targetAngle));
        DashboardLayout layout = WebdashboardServer.getInstance(5800).getFirstConnectedLayout();
        if(layout != null) {
            try {
                headingController.setPID(Double.parseDouble(layout.getInputValue("heading_kp")), Double.parseDouble(layout.getInputValue("heading_ki")), Double.parseDouble(layout.getInputValue("heading_kd")));
            } catch (NumberFormatException e) {
                e.printStackTrace();
            }
        }

        double limitedDrive = driveLimit.getLimitedInputValue(inputVector.magnitude);
        limitedDrive = driveLimit.getLimitedAccelerationValue(lastInput.vector.magnitude, limitedDrive);
        double limitedRot = rotationLimit.getLimitedInputValue(rot);
        limitedRot = rotationLimit.getLimitedAccelerationValue(lastInput.rot, limitedRot);
        Vector2d limitedVector = new Vector2d(limitedDrive, inputVector.angle, false); // Making another vector with the limited magnitude and the same angle
        limitedVector = limitedVector.rotate(-currentAngle); // This is necessary for field centric drive

        for (SwerveModule module : modules) {
            //module.drive(y, 0);
            module.rotateAndDrive(limitedVector, limitedRot, inDeadZone);
        }

        lastInput = new DriveInput(limitedVector, limitedRot);
    }

    public void drive(double x, double y, double rot) {
        drive(x, y, rot, true);
    }

    public static final class DriveLimits {
        public static final InputLimit NONE = new InputLimit() {
            @Override
            public double getLimitedInputValue(double currentValue, double... inputs) {
                return currentValue;
            }

            @Override
            public double getLimitedAccelerationValue(double lastValue, double currentValue) {
                return currentValue;
            }
        };
    }

    private static class DriveInput {
        final Vector2d vector;
        final double rot;

        DriveInput(Vector2d driveVector, double rot) {
            this.vector = driveVector;
            this.rot = rot;
        }
    }
}
