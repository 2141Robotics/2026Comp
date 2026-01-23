package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final TalonFX climberMotor;

    public Climber() {
        climberMotor = new TalonFX(ClimberConstants.climberMotorPort);
        init();
    }

    private void init() {
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
        climberMotor.setVoltage(0.0);
    }

    public void moveUp() {
        climberMotor.setVoltage(-ClimberConstants.CLIMBER_SPEED);
        
    }

    public void moveDown() {
        climberMotor.setVoltage(ClimberConstants.CLIMBER_SPEED);
        
    }

    

    @Override
    public void periodic() {
        super.periodic();
        
        climberMotor.setVoltage(0.0);
    }
}
