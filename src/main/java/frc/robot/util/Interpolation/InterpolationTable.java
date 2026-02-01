package frc.robot.util.Interpolation;
import java.util.List;

public class InterpolationTable {

    private final List<InterpPoint> points;

    public InterpolationTable(List<InterpPoint> points) {
        this.points = points;
    }

    public double get(double distance) {

        // Clamp low
        if (distance <= points.get(0).distance) {
            return points.get(0).value;
        }

        // Clamp high
        if (distance >= points.get(points.size() - 1).distance) {
            return points.get(points.size() - 1).value;
        }

        // Find surrounding points
        for (int i = 0; i < points.size() - 1; i++) {
            InterpPoint p1 = points.get(i);
            InterpPoint p2 = points.get(i + 1);

            if (distance >= p1.distance && distance <= p2.distance) {
                return interpolate(p1, p2, distance);
            }
        }

        // Should never happen
        return points.get(points.size() - 1).value;
    }

    private double interpolate(InterpPoint p1, InterpPoint p2, double distance) {
        double t = (distance - p1.distance) / (p2.distance - p1.distance);
        return p1.value + t * (p2.value - p1.value);
    }
}

