package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SpindexerIOSim implements SpindexerIO {
  private final DCMotorSim m_motorSim;
  public SpindexerIOSim() {
    DCMotor motorGearboxSim = DCMotor.getNEO(1);

    m_motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(motorGearboxSim, 0.0002, SpindexerConstants.indexerGearRatioSim),
        motorGearboxSim);
  }

  @Override
  public void setVoltage(double voltage) {
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
