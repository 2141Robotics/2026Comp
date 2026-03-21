package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricalConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final SparkMax intakeMotor;
    private final SparkMax intakeArmMotor;
    private boolean isArmDeployed = false;
    private double armDesiredPosition = IntakeConstants.INTAKE_ARM_MAX_POSITION;

    public Intake() {
        intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);
        intakeArmMotor = new SparkMax(IntakeConstants.INTAKE_ARM_MOTOR_PORT, MotorType.kBrushless);
        init();
    }

    private void init() {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kCoast);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.idleMode(IdleMode.kCoast);
        armConfig.smartCurrentLimit(ElectricalConstants.INTAKE_ARM_CURRENT_LIMIT);
        armConfig.closedLoop
            .p(IntakeConstants.INTAKE_ARM_KP)
            .i(IntakeConstants.INTAKE_ARM_KI)
            .d(IntakeConstants.INTAKE_ARM_KD);
        armConfig.inverted(true);
        intakeArmMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runIntake() {
        intakeMotor.set(IntakeConstants.INTAKE_RUN_SPEED);
    }

    public void spitIntake() {
        intakeMotor.set(-IntakeConstants.INTAKE_RUN_SPEED);
    }

    public void toggleDeployment() {
        isArmDeployed = !isArmDeployed;
        armDesiredPosition = isArmDeployed 
            ? IntakeConstants.INTAKE_ARM_MIN_POSITION 
            : IntakeConstants.INTAKE_ARM_MAX_POSITION;
    }

    public void moveOut() {
        armDesiredPosition += IntakeConstants.INTAKE_ARM_MANUAL_SPEED;
        // if (armDesiredPosition > IntakeConstants.INTAKE_ARM_MIN_POSITION) {
        //     armDesiredPosition = IntakeConstants.INTAKE_ARM_MIN_POSITION;
        // }
    }

    public void moveIn() {
        armDesiredPosition -= IntakeConstants.INTAKE_ARM_MANUAL_SPEED;
        // if (armDesiredPosition < IntakeConstants.INTAKE_ARM_MAX_POSITION) {
        //     armDesiredPosition = IntakeConstants.INTAKE_ARM_MAX_POSITION;
        // }
    }

    public void deployIntake() {
        isArmDeployed = true;
        armDesiredPosition = IntakeConstants.INTAKE_ARM_MIN_POSITION;
    }

    public void retractIntake() {
        isArmDeployed = false;
        armDesiredPosition = IntakeConstants.INTAKE_ARM_MAX_POSITION;
    }

    @Override
    public void periodic() {
        super.periodic();
        intakeMotor.set(0.0);
        intakeArmMotor.getClosedLoopController().setSetpoint(armDesiredPosition, ControlType.kPosition);
        SmartDashboard.putBoolean("Intake Arm Deployed", isArmDeployed);
        SmartDashboard.putNumber("Intake Arm Desired Position", armDesiredPosition);
        SmartDashboard.putNumber("Intake Arm Position", intakeArmMotor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Intake Motor Output", intakeMotor.getAppliedOutput());
    }

    public void setAngle(double d) {
        armDesiredPosition = d;
    }
}