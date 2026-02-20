package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import frc.robot.util.limits.MotorLim;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class ClimberIOReal implements ClimberIO {
  private final SparkFlex m_motor;
  private final AbsoluteEncoder m_encoder;
  private final SparkClosedLoopController m_positionController;

  public ClimberIOReal() {
    m_motor = SparkConfigurer.configSparkFlex(ClimberConstants.canId, SparkConstants.climberConfig);
    m_encoder = m_motor.getAbsoluteEncoder();
    m_positionController = m_motor.getClosedLoopController();
  }

  @Override
  public void setPosition(double position) {
    Logger.recordOutput("Subsystems/Climber/setpoint", position);
    double positionClamped = ClimberConstants.positionLimitsMeters.clampPosition(position);
    Logger.recordOutput("Subsystems/Climber/setpointClamped", positionClamped);
    m_positionController.setSetpoint(positionClamped, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setHeight(double height) {
    double position = (height - ClimberConstants.climberRetractedHeight) / Math.cos(ClimberConstants.climberAngleRadians);
    setPosition(position);
  }

  @Override
  public void setVoltage(double voltage) {
    Logger.recordOutput("Subsystems/Climber/desiredVoltage", voltage);
    double voltageClamped = MotorLim.clampVoltage(voltage);
    voltageClamped = ClimberConstants.positionLimitsMeters.clampOutput(m_encoder.getPosition(), voltageClamped);
    Logger.recordOutput("Subsystems/Climber/clampedVoltage", voltageClamped);
    m_motor.setVoltage(voltageClamped);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.positionMeters = m_encoder.getPosition();
    inputs.velocityMetersPerSec = m_encoder.getVelocity();
    inputs.heightMeters = ClimberConstants.climberRetractedHeight
        + inputs.positionMeters * Math.cos(ClimberConstants.climberAngleRadians);
    inputs.verticalVelocityMetersPerSec = inputs.velocityMetersPerSec
        * Math.cos(ClimberConstants.climberAngleRadians);
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.motorTemperature = m_motor.getMotorTemperature();
  }
}
