package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.webdashboard.DashboardLayout;

public class Limelight {

    private static Limelight instance = null;
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;

    private Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        table.getEntry("botpose");
    }

    // Read values periodically
    public void update() {
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        DashboardLayout.setNodeValue("limelight_x", "x: " + x);
        DashboardLayout.setNodeValue("limelight_y", "y: " + y);
        DashboardLayout.setNodeValue("limelight_area", "area: " + area);
    }

    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

}
