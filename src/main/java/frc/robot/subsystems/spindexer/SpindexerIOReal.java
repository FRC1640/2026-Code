package frc.robot.subsystems.spindexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class SpindexerIOReal implements SpindexerIO {
  private final SparkMax m_motor;
  private final RelativeEncoder m_encoder;

  public SpindexerIOReal() {
    m_motor = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(SpindexerConstants.indexerSparkCanId,
        SpindexerConstants.indexerSparkInverted));
    m_encoder = m_motor.getEncoder();
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.motorVelocity = m_encoder.getVelocity() * 2 * Math.PI / 60;
    inputs.motorVoltage = m_motor.getAppliedOutput();
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorTemperature = m_motor.getMotorTemperature();
  }
}
