package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricalConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.ShooterMath;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_PORT);
    private final SparkMax kickerMotor = new SparkMax(KickerConstants.KICKER_MOTOR_PORT, MotorType.kBrushless);
    private final SparkMax indexerMotor = new SparkMax(IndexerConstants.INDEXER_MOTOR_PORT, MotorType.kBrushless);

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    private boolean isShooting = false;
    private boolean adaptiveMode = false;

    // Timer used to measure time since last detected shot (current spike)
    private final Timer shotTimer = new Timer();
    // last sampled motor current (amps)
    private double lastShooterCurrent = 0.0;
    // threshold (amps) to consider a 'shot' current spike. Assumption: 10A is a reasonable default.


    private final SwerveSubsystem drivebase;
    private Translation2d target = null;

    public Shooter(SwerveSubsystem d) {
        this.drivebase = d;
        init();
        shotTimer.reset();
        shotTimer.start();
    }

    private void init() {
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.Slot0.kP = ShooterConstants.SHOOTER_KP;
        shooterConfig.Slot0.kI = ShooterConstants.SHOOTER_KI;
        shooterConfig.Slot0.kD = ShooterConstants.SHOOTER_KD;
        shooterConfig.Slot0.kV = ShooterConstants.SHOOTER_KV;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = ElectricalConstants.SHOOTER_CURRENT_LIMIT;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
        shooterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
        shooterMotor.getConfigurator().apply(shooterConfig);

        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        kickerConfig.idleMode(IdleMode.kCoast);
        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig indexerConfig = new SparkMaxConfig();
        indexerConfig.idleMode(IdleMode.kCoast);
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setShooterRPM(double rpm) {
        double rps = rpm / 60.0;
        shooterMotor.setControl(velocityControl.withVelocity(rps).withEnableFOC(true));
    }

    public void stop() {
        shooterMotor.stopMotor();
        kickerMotor.stopMotor();
        indexerMotor.stopMotor();
    }

    public double getRPM() {
        return shooterMotor.getVelocity().getValue().baseUnitMagnitude() * 60.0;
    }

    public void toggleShooting() {
        resetShotTimer();
        isShooting = !isShooting;
        adaptiveMode = false;
    }

    public void toggleAdaptiveShooting() {
        resetShotTimer();
        isShooting = !isShooting;
        adaptiveMode = true;
    }

    public void setTarget(Translation2d t) {
        this.target = t;
    }

    @Override
    public void periodic() {
        super.periodic();
        double desiredSpeed = 0.0;
        // sample current and detect spikes indicating a shot
        double shooterCurrent = shooterMotor.getStatorCurrent().getValueAsDouble();
        // if we cross the threshold from below -> above, consider that a shot and reset the timer
        if (shooterCurrent > ShooterConstants.CURRENT_SPIKE_THRESHOLD && lastShooterCurrent <= ShooterConstants.CURRENT_SPIKE_THRESHOLD) {
            resetShotTimer();
        }
        lastShooterCurrent = shooterCurrent;
        if (isShooting) {
            if (adaptiveMode) {
                double adaptiveRPM = ShooterMath.calculateAdaptiveShooterRPM(drivebase.getPose(), drivebase.getRobotVelocity(), this.target);
                setShooterRPM(adaptiveRPM);
                desiredSpeed = adaptiveRPM;
            } else {
                SubsystemStates.outsideShooterRange = false;
                setShooterRPM(ShooterConstants.SHOOTER_DEFAULT_RPM);
                desiredSpeed = ShooterConstants.SHOOTER_DEFAULT_RPM;
            }
            kickerMotor.set(KickerConstants.KICKER_SPEED);
            indexerMotor.set(IndexerConstants.INDEXER_SPEED);
        } else {
            stop();
        }
        SmartDashboard.putNumber("Shooter Desired RPM", desiredSpeed);
        SmartDashboard.putNumber("Shooter Actual RPM", getRPM());
        SmartDashboard.putNumber("Shooter Current (A)", shooterCurrent);
        SmartDashboard.putNumber("Time Since Last Shot (s)", timeSinceLastShot());
        SmartDashboard.putString("Target", target != null ? String.format("(%.2f, %.2f)", target.getX(), target.getY()) : "None");
        SmartDashboard.putNumber("Kicker Output", kickerMotor.getAppliedOutput());
        SmartDashboard.putNumber("Indexer Output", indexerMotor.getAppliedOutput());
    }

    private void resetShotTimer() {
        shotTimer.reset();
        shotTimer.start();
    }

    public double timeSinceLastShot() {
        // Return elapsed whole seconds since the last detected shot (current spike)
        return shotTimer.get();
    }
}