package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ShooterConstants;

public class ShooterMath {
    //Field measurements in inches, converted to meters
    private static final Translation2d blueHubPosition = new Translation2d((158.6 + (47.0 / 2.0)) * 0.0254, (317.7/2.0) * 0.0254);
    private static final Translation2d redHubPosition = new Translation2d((651.2 - (158.6 + (47 / 2.0))) * 0.0254, (317.7/2.0) * 0.0254);

    public static double calculateAdaptiveShooterRPM(Pose2d robotPose, ChassisSpeeds robotRelativeVelocity) {
        Translation2d requiredVelocity = calculateRequiredVelocity(robotPose, robotRelativeVelocity);
        return calculateRequiredRPM(requiredVelocity);
    }
    public static double calculateAdaptiveTurretAngle(Pose2d robotPose, ChassisSpeeds robotRelativeVelocity) {
        Translation2d requiredVelocity = calculateRequiredVelocity(robotPose, robotRelativeVelocity);
        return requiredVelocity.getAngle().getDegrees();
    }

    private static Translation2d calculateRequiredVelocity(Pose2d robotPose, ChassisSpeeds robotRelativeVelocity) {
        Pose2d shooterCenter = calculateShooterCenter(robotPose);
        double distance = calculateDistanceToHub(shooterCenter);
        SmartDashboard.putNumber("Distance to Alliance Hub", distance);
        double RPM = ShooterConstants.shooterTable.get(distance);
        Translation2d initialVelocity = calculateInitialVelocity(RPM, robotPose);
        Translation2d robotVelocity = new Translation2d(robotRelativeVelocity.vxMetersPerSecond, robotRelativeVelocity.vyMetersPerSecond).rotateBy(robotPose.getRotation());
        return initialVelocity.minus(robotVelocity);
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
    private static double calculateAngleToHub(Pose2d robotPose) {
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
        Translation2d shooterOffset = new Translation2d(ShooterConstants.SHOOTER_OFFSET, 0).rotateBy(robotPose.getRotation());
        return new Pose2d(robotPose.getTranslation().plus(shooterOffset), robotPose.getRotation());
    }

    /**
     * @param RPM The RPM the shooter would need to score at the given distance with no robot velocity
     */
    private static Translation2d calculateInitialVelocity(double RPM, Pose2d robotPose){
        double wheelRotationsPerSecond = RPM / 60.0;
        double magnitude = ShooterConstants.SHOOTER_WHEEL_CIRCUMFRENCE * wheelRotationsPerSecond * ShooterConstants.SHOOTER_EFFICIENCY;
        double angleToHub = Math.toRadians(calculateAngleToHub(robotPose));
        return new Translation2d(magnitude * Math.cos(angleToHub),
                    magnitude * Math.sin(angleToHub));
    }

    private static double calculateRequiredRPM(Translation2d requiredVelocity) {
        double wheelRotationsPerSecond = requiredVelocity.getNorm() / (ShooterConstants.SHOOTER_WHEEL_CIRCUMFRENCE * ShooterConstants.SHOOTER_EFFICIENCY);
        return wheelRotationsPerSecond * 60.0;
    }

    
    private static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

}
