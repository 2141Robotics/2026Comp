package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.ShooterMath;

public class Turret extends SubsystemBase {
    private final TalonFX turretMotor;

    private boolean adaptiveMode = false;

    private double turretDesiredAngle = 0.0;

    private final SwerveSubsystem drivebase;

    public Turret(SwerveSubsystem d) {
        this.drivebase = d;
        turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_PORT);
        init();
    }

    private void init() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = TurretConstants.TURRET_KP;
        config.Slot0.kI = TurretConstants.TURRET_KI;
        config.Slot0.kD = TurretConstants.TURRET_KD;
        config.Feedback.SensorToMechanismRatio = TurretConstants.TURRET_GEAR_RATIO;
        turretMotor.getConfigurator().apply(config);
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
        turretMotor.setPosition(0.0);
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

    @Override
    public void periodic() {
        super.periodic();
        if(adaptiveMode){
            turretDesiredAngle = ShooterMath.calculateAdaptiveTurretAngle(drivebase.getPose(), drivebase.getRobotVelocity());
            SmartDashboard.putNumber("Angle To Hub", turretDesiredAngle);
        }
        if(turretDesiredAngle > TurretConstants.TURRET_MAX_ANGLE){
            turretDesiredAngle = TurretConstants.TURRET_MAX_ANGLE;
            if(adaptiveMode){
                SubsystemStates.outsideTurretRange = true;
            }
        } else if(turretDesiredAngle < TurretConstants.TURRET_MIN_ANGLE){
            turretDesiredAngle = TurretConstants.TURRET_MIN_ANGLE;
            if(adaptiveMode){
                SubsystemStates.outsideTurretRange = true;
            }
        } else{
            SubsystemStates.outsideTurretRange = false;
        }
        //CONVERTS DEGREES TO ROTATIONS
        turretMotor.setControl(new PositionDutyCycle(turretDesiredAngle * MathConstants.DEGREES_TO_ROTATIONS).withEnableFOC(true));
        
        SmartDashboard.putNumber("Turret Angle", turretDesiredAngle);
    }
}