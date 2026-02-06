package frc.robot.subsystems.kicker;

import com.revrobotics.spark.SparkMax;

import frc.robot.util.spark.SparkConstants;
import frc.robot.util.limits.MotorLim;
import frc.robot.util.spark.SparkConfigurer;

public class KickerIOReal implements KickerIO {
  private final SparkMax m_motor;

  public KickerIOReal() {
    m_motor = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(KickerConstants.canId, false));
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(MotorLim.clampVoltage(voltage));
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.motorTemperature = m_motor.getMotorTemperature();
  }
}
