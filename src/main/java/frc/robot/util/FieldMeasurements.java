package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldMeasurements {

    // Field measurements in inches, converted to meters
    private static final Translation2d blueHubPosition = new Translation2d((158.6 + (47.0 / 2.0)) * 0.0254,
            (317.7 / 2.0) * 0.0254);
    private static final Translation2d redHubPosition = new Translation2d((651.2 - (158.6 + (47 / 2.0))) * 0.0254,
            (317.7 / 2.0) * 0.0254);

    private static final Translation2d bluePassingPositionLeft = null;
    private static final Translation2d bluePassingPositionRight = null;
    private static final Translation2d redPassingPositionLeft = null;
    private static final Translation2d redPassingPositionRight = null;

    private static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    public static Translation2d getHubPosition() {
        if (isRedAlliance()) {
            return redHubPosition;
        } else {
            return blueHubPosition;
        }
    }

    public static Translation2d getPassingPosition(Translation2d robotPosition) {
        if (isRedAlliance()) {
            if (robotPosition.getDistance(redPassingPositionLeft) < robotPosition
                    .getDistance(redPassingPositionRight)) {
                return redPassingPositionLeft;
            } else {
                return redPassingPositionRight;
            }
        } else {
            if (robotPosition.getDistance(bluePassingPositionLeft) < robotPosition
                    .getDistance(bluePassingPositionRight)) {
                if (robotPosition.getDistance(bluePassingPositionLeft) < robotPosition
                        .getDistance(bluePassingPositionRight)) {
                    return bluePassingPositionLeft;
                } else {
                    return bluePassingPositionRight;
                }
            }
        }
        System.out.println("Error in fieldmeasurements.java: could not determine passing position");
        return null;
    }
}
