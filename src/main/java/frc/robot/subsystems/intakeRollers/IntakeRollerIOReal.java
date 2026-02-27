package frc.robot.subsystems.intakeRollers;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.util.limits.VoltageLim;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class IntakeRollerIOReal implements IntakeRollerIO {
  private final SparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_velocityController;

  public IntakeRollerIOReal() {
    m_motor = SparkConfigurer.configSparkMax(IntakeRollerConstants.canID, SparkConstants.intakeRollerConfig);
    m_encoder = m_motor.getEncoder();
    m_velocityController = m_motor.getClosedLoopController();
  }

  @Override
  public void setVelocityRadPerSec(double velocityRadPerSec) {
    Logger.recordOutput("Subsystems/IntakeRollers/setpointVelocityRadPerSec", velocityRadPerSec);
    double velocityRPM = velocityRadPerSec * 60 / (2 * Math.PI);
    Logger.recordOutput("Subsystems/IntakeRollers/setpointVelocityRPM", velocityRPM);
    m_velocityController.setSetpoint(velocityRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVoltage(double voltage) {
    Logger.recordOutput("Subsystems/IntakeRollers/desiredVoltage", voltage);
    double voltageClamped = VoltageLim.clampVoltage(voltage);
    Logger.recordOutput("Subsystems/IntakeRollers/setpointVoltage", voltageClamped);
    m_motor.setVoltage(voltageClamped);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.motorTemperatureCelsius = m_motor.getMotorTemperature(); // degrees celsius
    inputs.motorCurrent = m_motor.getOutputCurrent(); // amps
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage(); // volts
    inputs.encoderVelocityRadiansPerSecond = m_encoder.getVelocity() * 2 * Math.PI / 60; // rad/s
  }
}
