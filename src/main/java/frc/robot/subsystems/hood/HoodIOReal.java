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
  private double encoderOffset = 0;

  public HoodIOReal() {
    m_motor = SparkConfigurer.configSparkMax(HoodConstants.canId, SparkConstants.hoodConfig);
    m_encoder = m_motor.getAbsoluteEncoder();
    m_motorController = m_motor.getClosedLoopController();
  }

  @Override
  public void setAngleRadians(double angle) {
    Logger.recordOutput("Subsystems/Hood/setpointRadians", angle);
    Logger.recordOutput("Subsystems/Hood/setpointDegrees", angle * 180 / Math.PI);
    double angleAdjusted = HoodConstants.angleLimitsRadians.clampPosition(angle);
    // TODO control type
    m_motorController.setSetpoint(radiansToEncoderCount(angleAdjusted), ControlType.kPosition,
        ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVoltage(double voltage) {
    Logger.recordOutput("Subsystems/Hood/desiredVoltage", voltage);
    double voltageClamped = VoltageLim.clampVoltage(voltage);
    voltageClamped = HoodConstants.angleLimitsRadians.clampOutput(getHoodAngleWithHorizontalRadians(),
        voltageClamped);
    Logger.recordOutput("Subsystems/Hood/clampedVoltage", voltageClamped);
    m_motor.setVoltage(voltageClamped);
  }

  @Override
  public void resetEncoder() {
    encoderOffset = m_encoder.getPosition() - HoodConstants.hoodEncoderManualOffset;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.angleHorizontalRadians = getHoodAngleWithHorizontalRadians();
    inputs.angleVerticalRadians = Math.PI / 2 - inputs.angleHorizontalRadians;
    inputs.angularVelocityRadPerSec = m_encoder.getVelocity() * HoodConstants.hoodEncoderToAngleRatio;
    inputs.angleHorizontalDegrees = inputs.angleHorizontalRadians * 180 / Math.PI;
    inputs.angleVerticalDegrees = inputs.angleVerticalRadians * 180 / Math.PI;
    inputs.angularVelocityDegreesPerSec = inputs.angularVelocityRadPerSec * 180 / Math.PI;
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.motorTemperatureCelsius = m_motor.getMotorTemperature();
    inputs.motorDrawJoules = inputs.motorCurrent * inputs.motorVoltage * 0.02;
    inputs.motorWattage = inputs.motorCurrent * inputs.motorVoltage;
    Logger.recordOutput("Subsystems/Hood/encoderPositionRaw", m_encoder.getPosition());
  }

  private double getHoodAngleWithHorizontalRadians() {
    return (m_encoder.getPosition() - encoderOffset - HoodConstants.hoodEncoderManualOffset)
        * HoodConstants.hoodEncoderToAngleRatio + HoodConstants.hoodZeroOffsetRadians;
  }

  private double radiansToEncoderCount(double horizontalAngleRadians) {
    return (horizontalAngleRadians - HoodConstants.hoodZeroOffsetRadians) / HoodConstants.hoodEncoderToAngleRatio
        + HoodConstants.hoodEncoderManualOffset;
  }
}
