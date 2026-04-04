package frc.robot.subsystems.spindexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import frc.robot.util.limits.ExponentialMovingAverage;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class SpindexerIOReal implements SpindexerIO {
  private final SparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final ExponentialMovingAverage m_currentEma;

  public SpindexerIOReal() {
    m_motor = SparkConfigurer.configSparkMax(SpindexerConstants.indexerSparkCanId, SparkConstants.spindexerConfig);
    m_encoder = m_motor.getEncoder();
    m_currentEma = new ExponentialMovingAverage(1, 1, () -> m_motor.getOutputCurrent());
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.motorVelocityRadPerSec = m_encoder.getVelocity() * 2 * Math.PI / 60;
    inputs.motorVelocityRPM = m_encoder.getVelocity();
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorTemperatureCelsius = m_motor.getMotorTemperature();
    inputs.isJammed = m_currentEma.get() > SpindexerConstants.jamCurrentThresh;
    inputs.motorTotalDrawJoules += inputs.motorCurrent * inputs.motorVoltage * 0.02;
    inputs.motorWattage = inputs.motorCurrent * inputs.motorVoltage;
  }
}
