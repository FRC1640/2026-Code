package frc.robot.subsystems.shooter.turret;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class TurretIOReal implements TurretIO {

  private final SparkMax m_motor;
  private final AbsoluteEncoder m_encoder;
  private final SparkClosedLoopController m_turretController;

  public TurretIOReal() {
    SparkConfiguration config = SparkConstants.getDefaultMax(TurretConstants.canId, false);
    m_motor = SparkConfigurer.configSparkMax(config);
    m_encoder = m_motor.getAbsoluteEncoder();
    m_turretController = m_motor.getClosedLoopController();
  }

  @Override
  public void setTurretState(double angle, double angularVelocity) {
    m_turretController.setSetpoint(angularVelocity, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.angle = m_encoder.getPosition() * 2 * Math.PI; // assuming 0-1 with zero point straight ahead
    inputs.angularVelocity = m_encoder.getVelocity() * 2 * Math.PI;
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorVoltage = m_motor.getBusVoltage() * m_motor.getAppliedOutput();
    inputs.motorTemperature = m_motor.getMotorTemperature();
  }
}
