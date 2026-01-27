package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final TalonFX ShooterMotor;

    public Shooter() {
        ShooterMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_PORT);
        init();
    }

    private void init() {
        ShooterMotor.setNeutralMode(NeutralModeValue.Coast);
        ShooterMotor.setVoltage(0.0);
    }

    public void shoot() {
        ShooterMotor.setVoltage(ShooterConstants.SHOOTER_SPEED);
        
        
    }

    

    @Override
    public void periodic() {
        super.periodic();
        
        ShooterMotor.setVoltage(0.0);
    }
}