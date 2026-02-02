// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Meter;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.util.Interpolation.InterpPoint;
import frc.robot.util.Interpolation.InterpolationTable;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  public static final class MathConstants {
    public static final double DEGREES_TO_ROTATIONS = 1.0 / 360.0;
  }

  public static final class AutonConstants {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(1, 0, 0);
    public static final PIDConstants ROTATION_PID = new PIDConstants(1, 0, 0);

    public static final Pose2d BLUE_STARTING_POSE = new Pose2d(new Translation2d(Meter.of(1),
        Meter.of(4)),
        Rotation2d.fromDegrees(0));

    public static final Pose2d RED_STARTING_POSE = new Pose2d(new Translation2d(Meter.of(16),
        Meter.of(4)),
        Rotation2d.fromDegrees(180));

  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static final double MAX_TRANSLATION_ACCELERATION = 3.0; // m/s^2
    public static final double MAX_ANGULAR_ACCELERATION = 360.0; // degrees/s^2
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;

    // Wtf does this do, maybe turn speed?
    public static final double TURN_CONSTANT = 6;

    public static final int DRIVE_CONTROLLER_PORT = 0;

    // Lowest speed the robot will go with bound trigger (left trigger as of 2026)
    // fully pressed
    public static final double MIN_INPUTTED_SPEED = 0.1;

    // Normal speed without any trigger input
    public static final double NORMAL_INPUTTED_SPEED = 0.3;

    // Highest speed the robot will go with bound trigger (right trigger as of 2026)
    // fully pressed
    public static final double MAX_INPUTTED_SPEED = 1.0;

    public static final double DRIVER_CONTROLLER_JOYSTICK_SCALER = 1.0;

  }

  public static class VisionConstants {

    /**
     * Ambiguity defined as a value between (0,1). Used in
     * {@link Vision#filterPose}.
     */
    public static final double MAX_AMBIGUITY = 0.2;

  }

  public static class ClimberConstants {
    public static final double CLIMBER_SPEED = 2; // Adjust as necessary
    public static final int CLIMBER_MOTOR_PORT = 31;
    public static final double CLIMBER_HEIGHT_MAX = 320.0; // max height of climber
    public static final double CLIMBER_HEIGHT_MIN = 0.0; // min height of climber
    public static final double CLIMBER_KP = 0.1;
    public static final double CLIMBER_KI = 0.0;
    public static final double CLIMBER_KD = 0.0;
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_PORT = 41;
    public static final int INTAKE_ARM_MOTOR_PORT = 42;
    public static final double INTAKE_SPEED = 1.0; // Adjust as necessary
    public static final double INTAKE_ARM_KP = 0.1;
    public static final double INTAKE_ARM_KI = 0;
    public static final double INTAKE_ARM_KD = 0;
    public static final double INTAKE_ARM_MAX_POSITION = 10.0;
    public static final double INTAKE_ARM_MIN_POSITION = 0;
  }

  public static class TurretConstants {
    public static final int TURRET_MOTOR_PORT = 61;
    public static final double TURRET_SPEED = 5.0; // Adjust as necessary
    public static final double TURRET_KP = 0;
    public static final double TURRET_KI = 0;
    public static final double TURRET_KD = 0;
    public static final double TURRET_GEAR_RATIO = 1;
    public static final double TURRET_MAX_ANGLE = 90;
    public static final double TURRET_MIN_ANGLE = -90;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_PORT = 51;
    public static final double SHOOTER_DEFAULT_RPM = 3000.0;
    // TODO populate with real values
    public static final InterpolationTable shooterDistanceTable = new InterpolationTable(List.of(
        new InterpPoint(1.5, 3200),
        new InterpPoint(2.0, 3600),
        new InterpPoint(2.5, 4100),
        new InterpPoint(3.0, 4700)));
    // How far the shooter is offset from the robot center, in meters
    public static final double SHOOTER_OFFSET = 10 * 0.0254;
  }

  public static class IndexerConstants {
    public static final int INDEXER_MOTOR_PORT = 61;
    public static final double INDEXER_SPEED = 1;
  }
  
  public static class KickerConstants {
    public static final int KICKER_MOTOR_PORT = 71;
    public static final double KICKER_SPEED = 1;
  }
}