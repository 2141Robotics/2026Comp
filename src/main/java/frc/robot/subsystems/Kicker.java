package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricalConstants;
import frc.robot.Constants.KickerConstants;

public class Kicker extends SubsystemBase {
    
    private final TalonFX kickerMotor;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    private double targetRPM = 0;

    private final DoubleSupplier shooterRPM;

    public Kicker(DoubleSupplier shooterRPM) {
        this.shooterRPM = shooterRPM;
        kickerMotor = new TalonFX(KickerConstants.KICKER_MOTOR_PORT);
        init();
    }

    private void init() {
        kickerMotor.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID gains (start small!)
        config.Slot0.kP = KickerConstants.KICKER_KP;
        config.Slot0.kI = KickerConstants.KICKER_KI;
        config.Slot0.kD = KickerConstants.KICKER_KD;
        config.Slot0.kV = KickerConstants.KICKER_KV; // Feedforward (VERY important for shooters)
        config.CurrentLimits.SupplyCurrentLimit = ElectricalConstants.KICKER_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        kickerMotor.getConfigurator().apply(config);
    }

    
    public void setRPM(double rpm) {
        kickerMotor.setControl(velocityControl.withVelocity(rpm/60.0).withEnableFOC(true));
        this.targetRPM = rpm;
    }

    // public void waitForShooterSpinup() {
    //     kickerMotor.setControl(velocityControl.withVelocity(0).withEnableFOC(true));
    // }

    public void stop() {
        kickerMotor.stopMotor();
        this.targetRPM = 0;
    }

    public double getTargetRPM() {
        return this.targetRPM;
    }

    @Override
    public void periodic() {
        super.periodic();
        // if(targetRPM != shooterRPM.getAsDouble()){
        //     waitForShooterSpinup();
        // }
    }
}
