package frc.robot.subsystems.indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class IndexerIOReal implements IndexerIO {
  private final SparkMax m_motor;
  private final RelativeEncoder m_encoder;
  public IndexerIOReal() {
    m_motor = SparkConfigurer.configSparkMax(
        SparkConstants.getDefaultMax(IndexerConstants.indexerSparkCanId, IndexerConstants.indexerSparkInverted));
    m_encoder = m_motor.getEncoder();
  }

  @Override
  public void setIndexerMotorVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.motorVelocity = m_encoder.getVelocity() * 2 * Math.PI / 60;
    inputs.motorVoltage = m_motor.getAppliedOutput();
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorTemperature = m_motor.getMotorTemperature();
  }
}
