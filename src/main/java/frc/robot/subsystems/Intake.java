
    package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.intakeMotorPort);
        init();
    }

    private void init() {
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
        intakeMotor.setVoltage(0.0);
    }

    public void intake() {
        intakeMotor.setVoltage(IntakeConstants.INTAKE_SPEED);
        
    }

    

    @Override
    public void periodic() {
        super.periodic();
        
        intakeMotor.setVoltage(0.0);
    }
}