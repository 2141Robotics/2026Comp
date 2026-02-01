package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    private final TalonFX turretMotor;

    private boolean adaptiveMode = false;

    private double turretDesiredAngle = 0.0;

    public Turret() {
        turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_PORT);
        init();
    }

    private void init() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = TurretConstants.TURRET_KP;
        config.Slot0.kI = TurretConstants.TURRET_KI;
        config.Slot0.kD = TurretConstants.TURRET_KD;
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
            //TODO calculate turretDesiredAngle based on target tracking
        }
        turretMotor.setControl(new PositionDutyCycle(turretDesiredAngle));
    }
}