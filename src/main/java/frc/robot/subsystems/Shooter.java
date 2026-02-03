package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.ShooterMath;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_PORT); // CAN ID

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    private boolean isShooting = false;

    private boolean adaptiveMode = false;

    private final SwerveSubsystem drivebase;

    private final Kicker kicker = new Kicker(this::getRPM);

    public Shooter(SwerveSubsystem d) {
        this.drivebase = d;
        init();
    }

    private void init() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID gains (start small!)
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12; // Feedforward (VERY important for shooters)

        shooterMotor.getConfigurator().apply(config);
    }

    public void setShooterRPM(double rpm) {
        double rps = rpm / 60.0; // rotations per second
        shooterMotor.setControl(velocityControl.withVelocity(rps));
        kicker.setRPM(rpm);
    }

    public void stop() {
        shooterMotor.stopMotor();
        kicker.stop();
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

    @Override
    public void periodic() {
        double desiredSpeed = 0.0;
        super.periodic();
        if (isShooting) {
            if (adaptiveMode) {
                double adaptiveRPM = ShooterMath.calculateAdaptiveShooterRPM(drivebase.getPose(), drivebase.getRobotVelocity());
                setShooterRPM(adaptiveRPM);
                desiredSpeed = adaptiveRPM;
            }else {
                setShooterRPM(ShooterConstants.SHOOTER_DEFAULT_RPM);
                desiredSpeed = ShooterConstants.SHOOTER_DEFAULT_RPM;
            }
        } else {
            stop();
        }
        SmartDashboard.putNumber("Shooter Commanded RPM", desiredSpeed);
    }
}