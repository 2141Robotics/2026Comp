package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldMeasurements {

    // Field measurements in inches, converted to meters
    private static final Translation2d blueHubPosition = new Translation2d((158.6 + (47.0 / 2.0)) * 0.0254,
            (317.7 / 2.0) * 0.0254);
    private static final Translation2d redHubPosition = new Translation2d((651.2 - (158.6 + (47 / 2.0))) * 0.0254,
            (317.7 / 2.0) * 0.0254);

    private static final Translation2d bluePassingPositionLeft = new Translation2d(3.5, 5.5);
    private static final Translation2d bluePassingPositionRight = new Translation2d(3.5, 2.5);
    private static final Translation2d redPassingPositionLeft = new Translation2d(16 - 3.5, 5.5);
    private static final Translation2d redPassingPositionRight = new Translation2d(16 - 3.5, 2.5);

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
                return bluePassingPositionLeft;
            } else {
                return bluePassingPositionRight;
            }
        }
    }
}
