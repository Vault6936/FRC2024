package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;

public class Vector2d {
    public final double x;
    public final double y;
    public final double magnitude;
    public final double angle;

    /**
     * If isCartesian is set to false, var1 and var2 will be interpreted respectively as r and theta.
     **/
    public Vector2d(double var1, double var2, boolean isCartesian) {
        if (isCartesian) {
            x = var1;
            y = var2;
            magnitude = calculateMagnitude(var1, var2);
            angle = calculateAngle(var1, var2);
        } else {
            x = var1 * Math.cos(var2);
            y = var1 * Math.sin(var2);
            magnitude = var1;
            angle = var2 % (2 * Math.PI);
        }
    }

    public Vector2d(double x, double y) {
        this(x, y, true);
    }

    public Vector2d() {
        this(0, 0);
    }

    private double calculateMagnitude(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    private double calculateAngle(double x, double y) {
        if (x == 0.0) {
            if (y < 0) {
                return -Math.PI / 2;
            } else {
                return Math.PI / 2;
            }
        } else if (x >= 0.0) {
            return Math.atan(y / x);
        } else {
            return Math.atan(y / x) + Math.PI;
        }
    }

    public Vector2d rotate(double angle) {
        return new Vector2d(this.magnitude, this.angle + angle, false);
    }

    public Translation2d toTranslation2d() {
        return new Translation2d(x, y);
    }

}
