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
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.ShooterMath;

public class Turret extends SubsystemBase {
    private final SparkMax turretMotor;

    private boolean adaptiveMode = false;
    private double turretDesiredAngle = 0.0;

    private final SwerveSubsystem drivebase;
    private Translation2d target = null;

    public Turret(SwerveSubsystem d) {
        this.drivebase = d;
        turretMotor = new SparkMax(TurretConstants.TURRET_MOTOR_PORT, MotorType.kBrushless);
        init();
    }

    private void init() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit((int) ElectricalConstants.TURRET_CURRENT_LIMIT);
        config.encoder.positionConversionFactor(1.0 / TurretConstants.TURRET_GEAR_RATIO);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(TurretConstants.TURRET_KP)
            .i(TurretConstants.TURRET_KI)
            .d(TurretConstants.TURRET_KD);
        turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turretMotor.getEncoder().setPosition(0.0);
    }

    public void turnLeft() {
        turretDesiredAngle -= TurretConstants.TURRET_SPEED;
        adaptiveMode = false;
    }

    public void turnRight() {
        turretDesiredAngle += TurretConstants.TURRET_SPEED;
        adaptiveMode = false;
    }

    public void activateAdaptiveMode() {
        adaptiveMode = true;
    }

    public void setTarget(Translation2d t) {
        this.target = t;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (adaptiveMode) {
            turretDesiredAngle = ShooterMath.calculateAdaptiveTurretAngle(drivebase.getPose(), drivebase.getRobotVelocity(), target);
            double turretAngleError = turretDesiredAngle - ShooterMath.calculateAdaptiveTurretAngle(drivebase.getPose(), new ChassisSpeeds(), target);
            SmartDashboard.putNumber("Turret Angle Compensation", turretAngleError);
        }

        if (turretDesiredAngle > TurretConstants.TURRET_MAX_ANGLE) {
            turretDesiredAngle = TurretConstants.TURRET_MAX_ANGLE;
            if (adaptiveMode) SubsystemStates.outsideTurretRange = true;
        } else if (turretDesiredAngle < TurretConstants.TURRET_MIN_ANGLE) {
            turretDesiredAngle = TurretConstants.TURRET_MIN_ANGLE;
            if (adaptiveMode) SubsystemStates.outsideTurretRange = true;
        } else {
            SubsystemStates.outsideTurretRange = false;
        }

        // CONVERTS DEGREES TO ROTATIONS
        turretMotor.getClosedLoopController().setSetpoint(
            turretDesiredAngle * MathConstants.DEGREES_TO_ROTATIONS,
            ControlType.kPosition
        );

        SmartDashboard.putNumber("Turret Desired Angle", turretDesiredAngle);
        SmartDashboard.putNumber("Turret Set Angle", turretMotor.getAbsoluteEncoder().getPosition());
    }
}