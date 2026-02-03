package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
    
    private final TalonFX indexerMotor;

    public Indexer() {
        indexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR_PORT);
        init();
    }

    private void init() {
        indexerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void runIndexer() {
        if(!SubsystemStates.outsideShooterRange && !SubsystemStates.outsideTurretRange){
            indexerMotor.setControl(new DutyCycleOut(IndexerConstants.INDEXER_SPEED));
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        indexerMotor.setControl(new DutyCycleOut(0.0));
    }
}
