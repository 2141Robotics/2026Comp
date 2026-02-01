package frc.robot.util;

public class InterpPoint {
    public final double distance;
    public final double value; // RPM or v0

    public InterpPoint(double distance, double value) {
        this.distance = distance;
        this.value = value;
    }
}

