package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;



public class Climber extends SubsystemBase {
    private final TalonFX climberMotor;
    private double climberCurrentHeight = ClimberConstants.CLIMBER_HEIGHT_MIN;
    public Climber() {
        climberMotor = new TalonFX(ClimberConstants.climberMotorPort);
        init();
    }

    private void init() {
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = ClimberConstants.CLIMBER_kP;
        config.Slot0.kI = ClimberConstants.CLIMBER_kI;
        config.Slot0.kD = ClimberConstants.CLIMBER_kD;
        climberMotor.getConfigurator().apply(config);
    }

    public void moveUp() {
        climberCurrentHeight-=ClimberConstants.CLIMBER_SPEED;
        if (climberCurrentHeight > ClimberConstants.CLIMBER_HEIGHT_MAX) {
            climberCurrentHeight = ClimberConstants.CLIMBER_HEIGHT_MAX;
        }
    }

    public void moveDown() {
        climberCurrentHeight-=ClimberConstants.CLIMBER_SPEED;
        if (climberCurrentHeight < ClimberConstants.CLIMBER_HEIGHT_MIN) {
            climberCurrentHeight = ClimberConstants.CLIMBER_HEIGHT_MIN;
        }
        
        
    }

    

    @Override
    public void periodic() {
        super.periodic();
        climberMotor.setControl(new PositionDutyCycle(climberCurrentHeight));
    }
}
