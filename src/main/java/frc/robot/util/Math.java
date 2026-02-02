package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Math {
    //Field measurements in inches, converted to meters
    private static final Translation2d blueHubPosition = new Translation2d((158.6 + (47.0 / 2.0)) * 0.0254, (317.7/2.0) * 0.0254);
    private static final Translation2d redHubPosition = new Translation2d((651.2 - (158.6 + (47 / 2.0))) * 0.0254, (317.7/2.0) * 0.0254);

    private static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    public static double calculateAdaptiveShooterRPM(Pose2d robotPose) {
        //TODO add velocity compensation
        double distance = calculateDistanceToHub(calculateShooterCenter(robotPose));
        SmartDashboard.putNumber("Distance to Alliance Hub", distance);
        return Constants.ShooterConstants.shooterTable.get(distance);
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

    /**
     * 
     * @param robotPose
     * @return angle in degrees from the robot's current position to the alliance hub
     */
    public static double calculateAngleToHub(Pose2d robotPose) {
        //TODO add velocity compensation
        Translation2d allianceHubPosition;
        if (isRedAlliance()) {
            allianceHubPosition = redHubPosition;
        } else {
            allianceHubPosition = blueHubPosition;
        }
        Pose2d shooterCenter = calculateShooterCenter(robotPose);
        Translation2d toHub = allianceHubPosition.minus(shooterCenter.getTranslation());
        double angleToHub = toHub.getAngle().getDegrees() - shooterCenter.getRotation().getDegrees();
        return angleToHub;
    }

    private static Pose2d calculateShooterCenter(Pose2d robotPose) {
        Translation2d shooterOffset = new Translation2d(Constants.ShooterConstants.SHOOTER_OFFSET, 0).rotateBy(robotPose.getRotation());
        return new Pose2d(robotPose.getTranslation().plus(shooterOffset), robotPose.getRotation());
    }
}
