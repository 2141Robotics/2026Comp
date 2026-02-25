package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricalConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final TalonFX intakeArmMotor;
    private boolean isArmDeployed = false;
    private double armDesiredPosition = IntakeConstants.INTAKE_ARM_MAX_POSITION;

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_PORT);
        intakeArmMotor = new TalonFX(IntakeConstants.INTAKE_ARM_MOTOR_PORT);
        init();
    }

    private void init() {
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);

        intakeArmMotor.setNeutralMode(NeutralModeValue.Brake);
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        armConfig.Slot0.kP = IntakeConstants.INTAKE_ARM_KP;
        armConfig.Slot0.kI = IntakeConstants.INTAKE_ARM_KI;
        armConfig.Slot0.kD = IntakeConstants.INTAKE_ARM_KD;
        armConfig.CurrentLimits.SupplyCurrentLimit = ElectricalConstants.INTAKE_ARM_CURRENT_LIMIT;
        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeArmMotor.getConfigurator().apply(armConfig);
    }

    public void runIntake() {
        intakeMotor.setControl(new DutyCycleOut(IntakeConstants.INTAKE_SPEED).withEnableFOC(true));
    }

    public void toggleDeployment() {
        isArmDeployed = !isArmDeployed;
        if (isArmDeployed) {
            armDesiredPosition = IntakeConstants.INTAKE_ARM_MIN_POSITION;
        } else {
            armDesiredPosition = IntakeConstants.INTAKE_ARM_MAX_POSITION;
        }
    }

    public void moveOut(){
        armDesiredPosition -= IntakeConstants.INTAKE_SPEED;
        if (armDesiredPosition < IntakeConstants.INTAKE_ARM_MIN_POSITION) {
            armDesiredPosition = IntakeConstants.INTAKE_ARM_MIN_POSITION;
        }
    }

    public void moveIn(){
        armDesiredPosition += IntakeConstants.INTAKE_SPEED;
        if (armDesiredPosition > IntakeConstants.INTAKE_ARM_MAX_POSITION) {
            armDesiredPosition = IntakeConstants.INTAKE_ARM_MAX_POSITION;
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        intakeMotor.setControl(new DutyCycleOut(0.0).withEnableFOC(true));
        intakeArmMotor.setControl(new PositionDutyCycle(armDesiredPosition).withEnableFOC(true));
    }
}