package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkMax;

import frc.robot.util.spark.SparkConstants;
import frc.robot.util.limits.MotorLim;
import frc.robot.util.spark.SparkConfigurer;

public class HopperIOReal implements HopperIO {
  private final SparkMax m_motor;

  public HopperIOReal() {
    m_motor = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(HopperConstants.canId, false));
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(MotorLim.clampVoltage(voltage));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.motorTemperature = m_motor.getMotorTemperature();
  }
}
