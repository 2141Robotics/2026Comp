package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // Indexer closed-loop controller
    private final SparkClosedLoopController indexerController;

    private boolean isShooting = false;
    private boolean adaptiveMode = false;

    private final Timer shotTimer = new Timer();
    private double lastShooterCurrent = 0.0;

    private final SwerveSubsystem drivebase;
    private Translation2d target = null;
    private int jigglingTimer = 0; // Timer for indexer jiggle state

    private int nudgeOffset;

    public Shooter(SwerveSubsystem d) {
        this.drivebase = d;
        indexerController = indexerMotor.getClosedLoopController();
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
        kickerConfig.inverted(true);
        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ── Indexer config (closed-loop velocity, 2026 API) ──────────────────
        SparkMaxConfig indexerConfig = new SparkMaxConfig();
        indexerConfig
            .idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(ElectricalConstants.INDEXER_CURRENT_LIMIT)
            .secondaryCurrentLimit(ElectricalConstants.INDEXER_CURRENT_LIMIT + 20);

        // FeedForwardConfig.kV replaces the deprecated velocityFF()
        // kV = 1 / NEO_VORTEX_FREE_SPEED_RPM = 1/6784 ≈ 0.000147
        FeedForwardConfig indexerFF = new FeedForwardConfig()
            .kV(IndexerConstants.INDEXER_KV);

        indexerConfig.closedLoop
            .pid(IndexerConstants.INDEXER_KP,
                 IndexerConstants.INDEXER_KI,
                 IndexerConstants.INDEXER_KD)
            .iZone(IndexerConstants.INDEXER_IZONE)
            .outputRange(-1.0, 1.0)
            .apply(indexerFF);  // attach feedforward

        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // ─────────────────────────────────────────────────────────────────────
    }

    public void setShooterRPM(double rpm) {
        double rps = (nudgeOffset + rpm) / 60.0;
        shooterMotor.setControl(velocityControl.withVelocity(rps).withEnableFOC(true));
    }

    public void stop() {
        shooterMotor.stopMotor();
        kickerMotor.stopMotor();
        indexerMotor.stopMotor();
    }

    public double getRPM() {
        return shooterMotor.getVelocity().getValue().baseUnitMagnitude() * 10;
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
        double shooterCurrent = shooterMotor.getStatorCurrent().getValueAsDouble();
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

            // ── Indexer: closed-loop velocity with jiggle on jam ─────────────
            // if (indexerMotor.getOutputCurrent() > ElectricalConstants.INDEXER_CURRENT_JIGGLE_LIMIT && jigglingTimer == 0) {
            //     jigglingTimer = 40;
            // }
            // if (jigglingTimer > 0) {
            //     jigglingTimer--;
            //     System.out.println("Indexer Jiggle Detected");
            //     indexerMotor.set(-0.3);
            // } else {
                indexerMotor.set(0.6);
            // }
            // ─────────────────────────────────────────────────────────────────

        } else {
            stop();
        }

        SmartDashboard.putNumber("Shooter Desired RPM", desiredSpeed + nudgeOffset);
        SmartDashboard.putNumber("Shooter Actual RPM", getRPM());
        SmartDashboard.putNumber("Shooter Current (A)", shooterCurrent);
        SmartDashboard.putNumber("Time Since Last Shot (s)", timeSinceLastShot()); 
        if(target != null) {
            SmartDashboard.putNumberArray("Target", new double[]{
                target.getX(), target.getY(), 0.0
            });
        }
        SmartDashboard.putNumber("Kicker Output", kickerMotor.getAppliedOutput());
        SmartDashboard.putNumber("Indexer Output", indexerMotor.getAppliedOutput());
        SmartDashboard.putNumber("Indexer Current (A)", indexerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Indexer Actual RPM", indexerMotor.getEncoder().getVelocity());
    }

    private void resetShotTimer() {
        shotTimer.reset();
        shotTimer.start();
    }

    public double timeSinceLastShot() {
        return shotTimer.get();
    }

    public void nudgeWeaker() {
        nudgeOffset -= ShooterConstants.nudgeAmount;
    }

    public void nudgeStronger() {
        nudgeOffset += ShooterConstants.nudgeAmount;
    }
}