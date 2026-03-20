package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricalConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.ShooterMath;

public class Turret extends SubsystemBase {
    private final SparkMax turretMotor;

    private boolean adaptiveMode = false;

    // Desired angle in degrees. Initialized to 0 — assumes the turret is
    // physically centered when the robot boots. If it isn't, call
    // zeroEncoder() after manually centering it, or add a limit switch home.
    private double turretDesiredAngle = 0.0;

    private final SwerveSubsystem drivebase;
    private Translation2d target = null;
    private boolean locked = false;

    public Turret(SwerveSubsystem d) {
        this.drivebase = d;
        turretMotor = new SparkMax(TurretConstants.TURRET_MOTOR_PORT, MotorType.kBrushless);
        init();
    }

    private void init() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit((int) ElectricalConstants.TURRET_CURRENT_LIMIT);

        // Convert motor rotations -> turret degrees via gear ratio.
        // With this factor, getPosition() returns degrees directly,
        // and setSetpoint() also expects degrees — no manual conversion needed.
        config.encoder
            .positionConversionFactor(360.0 / TurretConstants.TURRET_GEAR_RATIO);

        // Use the built-in relative encoder (no external absolute encoder present).
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(TurretConstants.TURRET_KP)
            .i(TurretConstants.TURRET_KI)
            .d(TurretConstants.TURRET_KD);

        config.inverted(true);

        turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Assume turret starts centered at 0 degrees. If it isn't physically
        // centered on boot, manually center it first then call zeroEncoder(),
        // or add a limit switch homing routine.
        turretMotor.getEncoder().setPosition(0.0);
        turretDesiredAngle = 0.0;
    }

    // -------------------------------------------------------------------------
    // Public interface
    // -------------------------------------------------------------------------

    public void turnLeft() {
        turretDesiredAngle += TurretConstants.TURRET_SPEED;
        adaptiveMode = false;
    }

    public void turnRight() {
        turretDesiredAngle -= TurretConstants.TURRET_SPEED;
        adaptiveMode = false;
    }

    public void activateAdaptiveMode() {
        adaptiveMode = true;
    }

    public void setTarget(Translation2d t) {
        this.target = t;
    }

    /**
     * Zeros the encoder at the current physical position.
     * Call this when the turret is known to be at 0 degrees (e.g. after homing).
     */
    public void zeroEncoder() {
        turretMotor.getEncoder().setPosition(0.0);
        turretDesiredAngle = 0.0;
    }

    /** Returns the turret's current angle in degrees as seen by the PID. */
    public double getActualAngleDegrees() {
        // positionConversionFactor already converts ticks to degrees.
        return turretMotor.getEncoder().getPosition();
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        super.periodic();

        // --- Adaptive (field-relative target tracking) mode ------------------
        if (adaptiveMode && target != null) {
            turretDesiredAngle = ShooterMath.calculateAdaptiveTurretAngle(
                drivebase.getPose(),
                drivebase.getRobotVelocity(),
                target
            );

            double staticAngle = ShooterMath.calculateAdaptiveTurretAngle(
                drivebase.getPose(),
                new ChassisSpeeds(),
                target
            );
            SmartDashboard.putNumber("Turret Angle Compensation", turretDesiredAngle - staticAngle);
        }

        // --- Soft limits -----------------------------------------------------
        if (turretDesiredAngle > TurretConstants.TURRET_MAX_ANGLE) {
            turretDesiredAngle = TurretConstants.TURRET_MAX_ANGLE;
            if (adaptiveMode) SubsystemStates.outsideTurretRange = true;
        } else if (turretDesiredAngle < TurretConstants.TURRET_MIN_ANGLE) {
            turretDesiredAngle = TurretConstants.TURRET_MIN_ANGLE;
            if (adaptiveMode) SubsystemStates.outsideTurretRange = true;
        } else {
            SubsystemStates.outsideTurretRange = false;
        }

        // --- Send setpoint to closed-loop controller -------------------------
        // positionConversionFactor handles the gear ratio and degree conversion,
        // so we pass degrees directly. The original code multiplied by
        // DEGREES_TO_ROTATIONS here, which was wrong — it would undo the
        // conversion factor and send a tiny fractional setpoint (~0.002),
        // making the motor think it was always nearly at its target.
        if (!locked) {
            turretMotor.getClosedLoopController().setSetpoint(
                turretDesiredAngle,
                ControlType.kPosition
            );
        }
        // --- Telemetry -------------------------------------------------------
        SmartDashboard.putNumber("Turret Desired Angle (deg)", turretDesiredAngle);
        SmartDashboard.putNumber("Turret Actual Angle (deg)",  getActualAngleDegrees());
        SmartDashboard.putNumber("Turret Angle Error (deg)",   turretDesiredAngle - getActualAngleDegrees());
        SmartDashboard.putBoolean("Turret Adaptive Mode",      adaptiveMode);
    }

    public void toggleLock(){
        locked = !locked;
    }
}