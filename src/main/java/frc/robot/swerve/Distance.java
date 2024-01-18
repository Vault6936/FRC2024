package frc.robot.swerve;

public class Distance {

    private final double distance;
    private final Unit unit;

    public Distance(double distance, Unit unit) {
        this.distance = distance;
        this.unit = unit;
    }

    public Distance(double distance) {
        this(distance, Unit.M);
    }

    public enum Unit {
        MM(0.001),
        CM(0.01),
        M(1),
        IN(0.0254),
        FT(0.3048);

        double value;
        Unit(double value) {
            this.value = value;
        }
    }

    public double getValue() {
        return distance;
    }

    public double getValueMM() {
        return distance * unit.value / Unit.MM.value;
    }

    public double getValueCM() {
        return distance * unit.value / Unit.CM.value;
    }

    public double getValueM() {
        return distance * unit.value;
    }

    public double getValueIN() {
        return distance * unit.value / Unit.IN.value;
    }

    public double getValueFT() {
        return distance * unit.value / Unit.FT.value;
    }

}
