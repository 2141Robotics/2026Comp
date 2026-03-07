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

    private final SwerveSubsystem drivebase;
    private Translation2d target = null;

    public Shooter(SwerveSubsystem d) {
        this.drivebase = d;
        init();
    }

    private void init() {
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.Slot0.kP = ShooterConstants.SHOOTER_KP;
        shooterConfig.Slot0.kI = ShooterConstants.SHOOTER_KI;
        shooterConfig.Slot0.kD = ShooterConstants.SHOOTER_KD;
        shooterConfig.Slot0.kV = ShooterConstants.SHOOTER_KV;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = ElectricalConstants.SHOOTER_CURRENT_LIMIT;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
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
        isShooting = !isShooting;
        adaptiveMode = false;
    }

    public void toggleAdaptiveShooting() {
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
        SmartDashboard.putNumber("Shooter Commanded RPM", desiredSpeed);
    }
}