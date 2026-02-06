package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElectricalConstants;

public class Climber extends SubsystemBase {
    private final TalonFX climberMotor;
    private double climberCurrentHeight = ClimberConstants.CLIMBER_HEIGHT_MIN;

    public Climber() {
        climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_PORT);
        init();
    }

    private void init() {
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
        TalonFXConfiguration config = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        config.Slot0.kP = ClimberConstants.CLIMBER_KP;
        config.Slot0.kI = ClimberConstants.CLIMBER_KI;
        config.Slot0.kD = ClimberConstants.CLIMBER_KD;
        config.CurrentLimits.SupplyCurrentLimit = ElectricalConstants.CLIMBER_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        climberMotor.getConfigurator().apply(config);
        climberMotor.setPosition(0);
    }

    public void moveUp() {
        climberCurrentHeight += ClimberConstants.CLIMBER_SPEED;
    }

    public void moveDown() {
        climberCurrentHeight -= ClimberConstants.CLIMBER_SPEED;
    }
    
    public boolean atHeight(){
        SmartDashboard.putNumber("ClimberActualHeight", climberCurrentHeight);
        return Math.abs(climberMotor.getPosition().getValueAsDouble() - climberCurrentHeight)/ climberCurrentHeight <= 0.05;
    }

    public void resetHeight(){
        climberMotor.setPosition(0);
        climberCurrentHeight = 0;
    }

    @Override
    public void periodic() {
        super.periodic();
        climberMotor.setControl(new PositionDutyCycle(climberCurrentHeight).withEnableFOC(true));
        SmartDashboard.putNumber("ClimberDesiredHeight", climberCurrentHeight);
        
        SmartDashboard.putNumber("ClimberCurrent",climberMotor.getStatorCurrent().getValueAsDouble());
    }

    public void climberUp() {
        climberCurrentHeight = ClimberConstants.CLIMBER_HEIGHT_MAX;
    }

    public void climberDown() {
        climberCurrentHeight = ClimberConstants.CLIMBER_HEIGHT_MIN;
    }
}
