package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final TalonFX climberMotor;

    private double desiredClimberHeight;

    public Climber() {
        climberMotor = new TalonFX(ClimberConstants.climberMotorPort);
        init();
    }

    private void init() {
        desiredClimberHeight = 0.0;
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 80;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kV = 0;
        limberMotor.getConfigurator().apply(config);
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void moveUp() {
        setDesiredClimberHeight(desiredClimberHeight + ClimberConstants.CLIMBER_SPEED);
    }

    public void moveDown() {
        setDesiredClimberHeight(desiredClimberHeight - ClimberConstants.CLIMBER_SPEED);
    }

    private void setDesiredClimberHeight(double height) {
        desiredClimberHeight = height;
    }

    @Override
    public void periodic() {
        super.periodic();
        climberMotor.setControl(new MotionMagicDutyCycle(desiredClimberHeight));
        
        //Units are unnecessary to calculate, as the value only matters relative to itself.
        SmartDashboard.putNumber("Climber Height (units unknown)", desiredClimberHeight);
    }
}
