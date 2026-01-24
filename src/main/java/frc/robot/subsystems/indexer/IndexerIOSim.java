package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim indexerSim;
  public IndexerIOSim() {
    DCMotor indexerMotorGearboxSim = DCMotor.getNEO(1);

    indexerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(indexerMotorGearboxSim, 0.0002,
        IndexerConstants.indexerGearRatioSim), indexerMotorGearboxSim);
  }

  @Override
  public void setIndexerMotorVoltage(double voltage) {
    indexerSim.setInputVoltage(voltage);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    indexerSim.update(0.02);

    inputs.indexerMotorVelocity = indexerSim.getAngularVelocityRadPerSec();
    inputs.indexerMotorVoltage = indexerSim.getInputVoltage();
    inputs.indexerMotorCurrent = indexerSim.getCurrentDrawAmps();
  }
}
