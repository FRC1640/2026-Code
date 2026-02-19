package frc.robot.subsystems.hood;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class HoodIOReal implements HoodIO {
  private final SparkMax m_motor;
  private final AbsoluteEncoder m_encoder;
  private final SparkClosedLoopController m_motorController;

  public HoodIOReal() {
    m_motor = SparkConfigurer.configSparkMax(HoodConstants.canId, SparkConstants.hoodConfig);

    m_encoder = m_motor.getAbsoluteEncoder();
    m_motorController = m_motor.getClosedLoopController();
  }

  @Override
  public void setAngleRad(double angle) { // TODO: Conversions!!!
    m_motorController.setSetpoint(angle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.0);
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.angleRadians = m_encoder.getPosition() * 2 * Math.PI;
    // TODO: same assumption as in TurretIOReal.java
    inputs.motorTemperatureCelsius = m_motor.getMotorTemperature();
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
  }
}
