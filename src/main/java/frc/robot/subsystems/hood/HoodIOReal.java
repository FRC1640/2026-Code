package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.util.limits.VoltageLim;
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
  public void setAngleRadians(double angle) {
    Logger.recordOutput("Subsystems/Hood/setpointRadians", angle);
    Logger.recordOutput("Subsystems/Hood/setpointDegrees", angle * 180 / Math.PI);
    double angleAdjusted = HoodConstants.angleLimitsRadians
        .clampPosition(angle - HoodConstants.hoodZeroOffsetRadians);
    // TODO control type
    m_motorController.setSetpoint(angleAdjusted, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVoltage(double voltage) {
    Logger.recordOutput("Subsystems/Hood/desiredVoltage", voltage);
    double voltageClamped = VoltageLim.clampVoltage(voltage);
    voltageClamped = HoodConstants.angleLimitsRadians.clampOutput(
        m_encoder.getPosition() * 2 * Math.PI + HoodConstants.hoodZeroOffsetRadians, voltageClamped);
    Logger.recordOutput("Subsystems/Hood/clampedVoltage", voltageClamped);
    m_motor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.angleRadians = m_encoder.getPosition() * 2 * Math.PI + HoodConstants.hoodZeroOffsetRadians;
    inputs.angularVelocityRadPerSec = m_encoder.getVelocity() * 2 * Math.PI;
    inputs.angleDegrees = inputs.angleRadians * 180 / Math.PI;
    inputs.angularVelocityDegreesPerSec = inputs.angularVelocityRadPerSec * 180 / Math.PI;
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.motorTemperatureCelsius = m_motor.getMotorTemperature();
  }
}
