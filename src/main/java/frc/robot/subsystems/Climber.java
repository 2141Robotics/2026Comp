package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 80;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kV = 0;
        climberMotor.getConfigurator().apply(config);
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void moveUp() {
        // setDesiredClimberHeight(desiredClimberHeight + ClimberConstants.CLIMBER_SPEED);
        climberMotor.setControl(new DutyCycleOut(-1));
    }

    public void moveDown() {
        // setDesiredClimberHeight(desiredClimberHeight - ClimberConstants.CLIMBER_SPEED);
        climberMotor.setControl(new DutyCycleOut(1));
    }

    

    @Override
    public void periodic() {
        super.periodic();
        climberMotor.setControl(new DutyCycleOut(0));
    }
}
