package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim m_motorSim;
  public IndexerIOSim() {
    DCMotor motorGearboxSim = DCMotor.getNEO(1);

    m_motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(motorGearboxSim, 0.0002, IndexerConstants.gearRatioSim),
        motorGearboxSim);
  }

  @Override
  public void setIndexerMotorVoltage(double voltage) {
    m_motorSim.setInputVoltage(voltage);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    m_motorSim.update(0.02);

    inputs.motorVelocity = m_motorSim.getAngularVelocityRadPerSec();
    inputs.motorVoltage = m_motorSim.getInputVoltage();
    inputs.motorCurrent = m_motorSim.getCurrentDrawAmps();
    inputs.motorTemperature = 0.0;
  }
}
