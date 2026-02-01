package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class Math {
    //Field measurements in inches, converted to meters
    private static final Translation2d blueHubPosition = new Translation2d(158.6 + (47 / 2.0) * 0.0254, 317.7 * 0.0254);
    private static final Translation2d redHubPosition = new Translation2d(651.2 - (158.6 + (47 / 2.0)) * 0.0254, 317.7 * 0.0254);

    private static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    public static double calculateAdaptiveShooterRPM(Pose2d robotPose) {
        //TODO add velocity compensation
        return Constants.ShooterConstants.shooterTable.get(calculateDistanceToHub(robotPose));
    }

    private static double calculateDistanceToHub(Pose2d robotPose) {
        Translation2d allianceHubPosition;
        if (isRedAlliance()) {
            allianceHubPosition = redHubPosition;
        } else {
            allianceHubPosition = blueHubPosition;
        }
        return robotPose.getTranslation().getDistance(allianceHubPosition);
    }
}
