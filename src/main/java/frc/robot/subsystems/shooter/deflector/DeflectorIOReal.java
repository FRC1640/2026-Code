package frc.robot.subsystems.shooter.deflector;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class DeflectorIOReal implements DeflectorIO {
  private final SparkMax m_motor;
  private final AbsoluteEncoder m_encoder;
  private final SparkClosedLoopController m_motorController;

  public DeflectorIOReal() {
    SparkConfiguration config = SparkConstants.getDefaultMax(DeflectorConstants.canId, false);
    m_motor = SparkConfigurer.configSparkMax(config);

    m_encoder = m_motor.getAbsoluteEncoder();
    m_motorController = m_motor.getClosedLoopController();
  }

  @Override
  public void setAngle(double angle) { // TODO: Conversions!!!
    m_motorController.setSetpoint(angle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.0);
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.set(voltage);
  }

  @Override
  public void setDeflectorMotorVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(DeflectorIOInputs inputs) {
    inputs.angle = m_encoder.getPosition() * 2 * Math.PI;
    // TODO: same assumption as in TurretIOReal.java
    inputs.motorTemperature = m_motor.getMotorTemperature();
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
  }
}
