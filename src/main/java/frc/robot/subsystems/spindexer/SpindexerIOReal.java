package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.util.limits.ExponentialMovingAverage;
import frc.robot.util.limits.VoltageLim;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class SpindexerIOReal implements SpindexerIO {
  private final SparkFlex m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_velocityController;
  private final ExponentialMovingAverage m_currentEma;

  public SpindexerIOReal() {
    m_motor = SparkConfigurer.configSparkFlex(SpindexerConstants.indexerSparkCanId, SparkConstants.spindexerConfig);
    m_encoder = m_motor.getEncoder();
    m_velocityController = m_motor.getClosedLoopController();
    m_currentEma = new ExponentialMovingAverage(1, 1, () -> m_motor.getOutputCurrent());
  }

  @Override
  public void setVelocityRPM(double velocityRPM) {
    Logger.recordOutput("Subsystems/Spindexer/setpointVelocityRPM", velocityRPM);
    Logger.recordOutput("Subsystems/Spindexer/setpointVelocityRadPerSec", velocityRPM * Math.PI / 30);
    m_velocityController.setSetpoint(velocityRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVoltage(double voltage) {
    Logger.recordOutput("Subsystems/Spindexer/desiredVoltage", voltage);
    double voltageClamped = VoltageLim.clampVoltage(voltage);
    Logger.recordOutput("Subsystems/Spindexer/setpointVoltage", voltageClamped);
    m_motor.setVoltage(voltageClamped);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.motorVelocityRadPerSec = m_encoder.getVelocity() * 2 * Math.PI / 60;
    inputs.motorVelocityRPM = m_encoder.getVelocity();
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorTemperatureCelsius = m_motor.getMotorTemperature();
    inputs.isJammed = m_currentEma.get() > SpindexerConstants.jamCurrentThresh;
    inputs.motorTotalEnergy += inputs.motorCurrent * inputs.motorVoltage * 0.02;
    inputs.motorPower = inputs.motorCurrent * inputs.motorVoltage;
  }
}
