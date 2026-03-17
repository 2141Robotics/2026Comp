package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

public class ShooterMath {

    public static double calculateAdaptiveShooterRPM(Pose2d robotPose, ChassisSpeeds robotRelativeVelocity,
            Translation2d target) {
        Translation2d requiredVelocity = calculateRequiredVelocity(robotPose, robotRelativeVelocity, target);
        return calculateRequiredRPM(requiredVelocity);
    }

    public static double calculateAdaptiveTurretAngle(Pose2d robotPose, ChassisSpeeds robotRelativeVelocity,
            Translation2d target) {
        Translation2d requiredVelocity = calculateRequiredVelocity(robotPose, robotRelativeVelocity, target);
        return requiredVelocity.getAngle().getDegrees();
    }

    private static Translation2d calculateRequiredVelocity(Pose2d robotPose, ChassisSpeeds robotRelativeVelocity,
            Translation2d target) {
        Pose2d shooterCenter = calculateShooterCenter(robotPose);
        double distance = calculateDistanceToTarget(shooterCenter, target);
        SmartDashboard.putNumber("Distance to Target", distance);
        double RPM = ShooterConstants.shooterTable.get(distance);
        Translation2d initialVelocity = calculateInitialVelocity(RPM, robotPose, target);
        Translation2d robotVelocity = new Translation2d(robotRelativeVelocity.vxMetersPerSecond,
                robotRelativeVelocity.vyMetersPerSecond).rotateBy(robotPose.getRotation());
        return initialVelocity.minus(robotVelocity);
    }
private static double calculateDistanceToTarget(Pose2d robotPose, Translation2d target) {
    SmartDashboard.putNumberArray("Target", new double[]{
        target.getX(), target.getY(), 0.0
    });
    return robotPose.getTranslation().getDistance(target);
}
    /**
     * 
     * @param robotPose
     * @return angle in degrees from the robot's current position to the alliance
     *         target
     */
    private static double calculateAngleToTarget(Pose2d robotPose, Translation2d target) {
        Pose2d shooterCenter = calculateShooterCenter(robotPose);
        Translation2d toTarget = target.minus(shooterCenter.getTranslation());
        double angleToTarget = toTarget.getAngle().getDegrees() - shooterCenter.getRotation().getDegrees();
        return angleToTarget;
    }

    private static Pose2d calculateShooterCenter(Pose2d robotPose) {
                
        return new Pose2d(robotPose.getTranslation().plus(ShooterConstants.SHOOTER_OFFSET.rotateBy(robotPose.getRotation())), robotPose.getRotation());
    }

    /**
     * @param RPM The RPM the shooter would need to score at the given distance with
     *            no robot velocity
     */
    private static Translation2d calculateInitialVelocity(double rpm, Pose2d robotPose, Translation2d target) {
        double magnitude = rpmToSpeed(rpm);
        double angleToTarget = Math.toRadians(calculateAngleToTarget(robotPose, target));
        return new Translation2d(magnitude * Math.cos(angleToTarget),
                magnitude * Math.sin(angleToTarget));
    }

    private static double calculateRequiredRPM(Translation2d requiredVelocity) {
        double requiredSpeed = requiredVelocity.getNorm();
        return speedToRPM(requiredSpeed);
    }

    private static double rpmToSpeed(double rpm) {
        double wheelRotationsPerSecond = rpm / 60.0;
        double magnitude = ShooterConstants.SHOOTER_WHEEL_CIRCUMFRENCE * wheelRotationsPerSecond
                * ShooterConstants.SHOOTER_EFFICIENCY;
        return magnitude;
    }

    private static double speedToRPM(double speedMetersPerSecond) {
        double wheelRotationsPerSecond = speedMetersPerSecond
                / (ShooterConstants.SHOOTER_WHEEL_CIRCUMFRENCE * ShooterConstants.SHOOTER_EFFICIENCY);
        return wheelRotationsPerSecond * 60;
    }
}
