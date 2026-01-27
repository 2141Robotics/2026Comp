package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    private final TalonFX turretMotor;

    public Turret() {
        turretMotor = new TalonFX(TurretConstants.turretMotorPort);
        init();
    }

    private void init() {
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
        turretMotor.setVoltage(0.0);
    }

    public void turnLeft() {
        turretMotor.setVoltage(TurretConstants.TURRET_SPEED);
        // turretMotor.positionDutyCycle.set(0.5);
        
    }

    public void turnRight() {
        turretMotor.setVoltage(-TurretConstants.TURRET_SPEED);
        
    }

    

    @Override
    public void periodic() {
        super.periodic();
        
        turretMotor.setVoltage(0.0);
    }
}