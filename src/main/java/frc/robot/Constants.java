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
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
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
    public static final double PI = Math.PI;
  }

  public static final class AutonConstants {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(1, 0, 0);
    public static final PIDConstants ROTATION_PID = new PIDConstants(.1, 0, 0);

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
    public static final double CLIMBER_HEIGHT_MAX = 51.5; // max height of climber
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
    public static final InterpolationTable shooterTable = new InterpolationTable(List.of(
        new InterpPoint(1.5, 3200),
        new InterpPoint(2.0, 3600),
        new InterpPoint(2.5, 4100),
        new InterpPoint(3.0, 4700)));
    // How far the shooter is offset from the robot center, in meters
    public static final double SHOOTER_OFFSET = 10 * 0.0254;
    // Diameter of the shooter wheel in meters
    public static final double SHOOTER_WHEEL_CIRCUMFRENCE = 4 * 0.0254 * MathConstants.PI;
    // Efficiency factor to account for losses, assumed to be linear
    public static final double SHOOTER_EFFICIENCY = 0.8;
    public static final double SHOOTER_KP = 0.1;
    public static final double SHOOTER_KI = 0;
    public static final double SHOOTER_KD = 0;
    public static final double SHOOTER_KV = 0.12;
  }

  public static class IndexerConstants {
    public static final int INDEXER_MOTOR_PORT = 61;
    public static final double INDEXER_SPEED = 1;
  }

  public static class KickerConstants {
    public static final int KICKER_MOTOR_PORT = 71;
    public static final double KICKER_SPEED = 1;
    public static final double KICKER_KP = 0.1;
    public static final double KICKER_KI = 0;
    public static final double KICKER_KD = 0;
    public static final double KICKER_KV = 0.12;
  }

  public static class ElectricalConstants {
    // Note the drive and steering motor current limits can be found in the YAGSL
    // .json files in deploy
    public static final double CLIMBER_CURRENT_LIMIT = 40;
    public static final double SHOOTER_CURRENT_LIMIT = 80;
    public static final double TURRET_CURRENT_LIMIT = 20;
    public static final double INTAKE_CURRENT_LIMIT = 40;
    public static final double INTAKE_ARM_CURRENT_LIMIT = 40;
    public static final double KICKER_CURRENT_LIMIT = 40;
    public static final double INDEXER_CURRENT_LIMIT = 40;
  }

  public static class LEDConstants {
    public static final double BLINK_ON_LENGTH = 1;

    // Amount of time the LEDs are off when blinking in seconds
    public static final double BLINK_OFF_LENGTH = 1;

    // Amount of cycles the LEDs blink for
    public static final int BLINK_CYCLES = 5;

    // The port number of the LED strip
    public static final int LED_PORT = 4;

    // The number of LEDs on the strip
    public static final int LED_COUNT = 288;

    // The offsets for the different LED strips
    // Note: The top LED strip is counted as one and is therefore given only one
    // offset at 105
    public static final int[] LED_STRIP_OFFSETS = new int[] {0, 105, 181};

    private static final Dimensionless LED_BRIGHTNESS = Percent.of(50);

    private static final Distance LEDPATTERN_DISTANCE = Meters.of(1.0 / 120);

    public static final Time BREATHE_LOOP_TIME = Time.ofBaseUnits(4, Second);

    public static final LEDPattern PATTERN_RED = LEDPattern.solid(Color.kRed).atBrightness(LED_BRIGHTNESS);
    public static final LEDPattern PATTERN_ORANGE = LEDPattern.solid(Color.kOrange).atBrightness(LED_BRIGHTNESS);
    public static final LEDPattern PATTERN_YELLOW = LEDPattern.solid(Color.kYellow).atBrightness(LED_BRIGHTNESS);
    public static final LEDPattern PATTERN_GREEN = LEDPattern.solid(Color.kGreen).atBrightness(LED_BRIGHTNESS);
    public static final LEDPattern PATTERN_BLUE = LEDPattern.solid(Color.kBlue).atBrightness(LED_BRIGHTNESS);
    public static final LEDPattern PATTERN_PURPLE = LEDPattern.solid(Color.kPurple).atBrightness(LED_BRIGHTNESS);
    public static final LEDPattern PATTERN_OFF = LEDPattern.kOff;

    // public static final LEDPattern PATTERN_DLS_GREEN =
    // LEDPattern.solid(new Color("#0F4D2A")).atBrightness(LED_BRIGHTNESS);

    public static final LEDPattern PATTERN_DLS_GREEN = LEDPattern.solid(Color.fromHSV(145, 78, 30))
        .atBrightness(LED_BRIGHTNESS);

    public static final LEDPattern PATTERN_UP = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kPurple,
        Color.kDarkBlue);
    public static final LEDPattern PATTERN_DOWN = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kDarkBlue,
        Color.kPurple);

    public static final LEDPattern PATTERN_SCROLL = LEDPattern
        .gradient(GradientType.kContinuous, Color.kDarkBlue, Color.kPurple)
        .scrollAtAbsoluteSpeed(InchesPerSecond.of(20), LEDPATTERN_DISTANCE);

    public static final LEDPattern PATTERN_FIRE = LEDPattern.solid(Color.kDarkRed);

    public static final int POLICE_SIREN_FREQUENCY = 40; // frequency played by motors in Hz
    public static final int POLICE_BLINK_SPEED = 3; // cycles per color change

    // 1 Seg1 On Seg2 Off
    // 2 Seg1 Off Seg2 Off
    // 3 Seg1 On Seg2 Off
    // 4 Seg1 Off Seg2 On
    // 5 Seg1 Off Seg2 Off
    // 6 Seg1 Off Seg2 On
    // 7 Seg1 On Seg2 Off
    // 8 Seg1 On Seg2 Off
    // 9 Seg1 Off Seg2 On
    // 10 Seg1 Off Seg2 On
    // 11 Seg1 On Seg2 Off
    // 12 Seg1 On Seg2 Off
    // 13 Seg1 Off Seg2 On
    // 14 Seg1 Off Seg2 On
    public static final boolean[][] POLICE_PATTERN = {
        { true, false },
        { true, false },
        { false, false },
        { false, false },
        { true, false },
        { true, false },
        { false, true },
        { false, true },
        { false, false },
        { false, false },
        { false, true },
        { false, true },
        { true, false },
        { true, false },
        { true, false },
        { true, false },
        { false, true },
        { false, true },
        { false, true },
        { false, true },
        { true, false },
        { true, false },
        { true, false },
        { true, false },
        { false, true },
        { false, true },
        { false, true },
        { false, true }
    };

    // The Police pattern is broken into eight sections
    // Blue + White + Blue + Blue + Red + Red + White + Red
    // This stat shortens the white segments by that many LEDs
    public static final int POLICE_WHITE_LENGTH_DIFFERENCE = 1;

    public static final LEDPattern PATTERN_POLICE = LEDPattern.solid(Color.kWhite);
    public static final LEDPattern PATTERN_POLICE_RED = LEDPattern.solid(Color.kRed);
    public static final LEDPattern PATTERN_POLICE_BLUE = LEDPattern.solid(Color.kBlue);

    public static final int DOTS_TRAIL_LENGTH = 10;

    // How long in cycles (20 ms intervals) it takes for a dot to travel
    // the length of the segment one way
    public static final double DOT_FREQUENCY_CYCLES = 110;

    // The color of the dots themselves
    public static final Color DOT_COLOR = Color.kWhite;
    // Just an object to refer to, this has no effect on the actual color
    public static final LEDPattern PATTERN_DOTS = LEDPattern.solid(Color.kWhite);
    // The color of the background
    public static final LEDPattern PATTERN_DOTS_BACKGROUND = LEDPattern.kOff;

    public static final LEDPattern PATTERN_RAINBOW_SCROLLING = LEDPattern.rainbow(255, 255)
        .scrollAtAbsoluteSpeed(InchesPerSecond.of(40), LEDPATTERN_DISTANCE);

    public static final LEDPattern PATTERN_PARTICLES = LEDPattern.solid(Color.kWhite);

    public static final LEDPattern PATTERN_AURORA = LEDPattern.solid(Color.kWhite);

    // Particle settings
    public static final double PARTICLE_SPAWN_CHANCE = 0.4;
    public static final double PARTICLE_SPLIT_CHANCE = 0.0;
    public static final double PARTICLE_DISAPPEAR_CHANCE = 0.001;
    public static final int PARTICLE_MAX_COUNT = 25;
    public static final Color PARTICLE_COLOR = Color.kWhite;
    public static final Color PARTICLE_BACKGROUND_COLOR = Color.kBlack; // Background is completely off
    public static final Color PARTICLE_EXPLOSION_COLOR = Color.kBlack;

  }
}