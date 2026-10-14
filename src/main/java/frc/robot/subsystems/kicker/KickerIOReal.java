package frc.robot.subsystems.kicker;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import frc.robot.util.limits.MotorLim;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class KickerIOReal implements KickerIO {
  private final SparkFlex m_motor;
  private final RelativeEncoder m_encoder;

  private final SparkClosedLoopController m_controller;

  public KickerIOReal() {
    m_motor = SparkConfigurer.configSparkFlex(KickerConstants.canId, SparkConstants.kickerConfig);
    m_encoder = m_motor.getEncoder();
    m_controller = m_motor.getClosedLoopController();
  }

  @Override
  public void setVelocity(double velocity) {
    m_controller.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(MotorLim.clampVoltage(voltage));
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.motorTemperatureCelsius = m_motor.getMotorTemperature();
    inputs.motorVelocityRadPerSec = m_encoder.getVelocity() * 2 * Math.PI / 60;
    inputs.motorVelocityRPM = m_encoder.getVelocity();

    inputs.motorTotalEnergy += inputs.motorCurrent * inputs.motorVoltage * 0.02;
    inputs.motorPower = inputs.motorVoltage * inputs.motorCurrent; // W
  }
}
