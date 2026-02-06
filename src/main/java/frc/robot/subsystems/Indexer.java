package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricalConstants;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
    
    private final TalonFX indexerMotor;

    public Indexer() {
        indexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR_PORT);
        init();
    }

    private void init() {
        indexerMotor.setNeutralMode(NeutralModeValue.Coast);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = ElectricalConstants.INDEXER_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerMotor.getConfigurator().apply(config);
    }

    public void runIndexer() {
        indexerMotor.setControl(new DutyCycleOut(IndexerConstants.INDEXER_SPEED).withEnableFOC(true));
    }

    @Override
    public void periodic() {
        super.periodic();
        indexerMotor.setControl(new DutyCycleOut(0.0).withEnableFOC(true));
    }
}
