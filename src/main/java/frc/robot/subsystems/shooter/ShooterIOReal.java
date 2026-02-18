package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class ShooterIOReal implements ShooterIO {
  private final SparkFlex m_leaderMotor;
  private final RelativeEncoder m_leaderEncoder;
  private final SparkFlex m_followerMotor;
  private final RelativeEncoder m_followerEncoder;

  private final SparkClosedLoopController m_motorController;

  public ShooterIOReal() {
    SparkConfiguration config = SparkConstants.getShooterFlex(ShooterConstants.canId, false);
    config.getInnerConfig()
      .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0004, 0, 0.003, ClosedLoopSlot.kSlot0)
        .pid(0.0006, 0, 0, ClosedLoopSlot.kSlot1)
        .pid(0.0001, 0, 0, ClosedLoopSlot.kSlot2)
        .pid(0.0001, 0, 0, ClosedLoopSlot.kSlot3)
      .feedForward
        .kV(0.002, ClosedLoopSlot.kSlot0).kA(0.0001, ClosedLoopSlot.kSlot0)
        .kV(0.002, ClosedLoopSlot.kSlot1).kA(0.002, ClosedLoopSlot.kSlot1)
        .kV(0.0017, ClosedLoopSlot.kSlot2)
        .kV(0.0019, ClosedLoopSlot.kSlot3);
    config.getInnerConfig()
      .closedLoop.maxMotion
        .maxAcceleration(4000, ClosedLoopSlot.kSlot0)
        .maxAcceleration(4000, ClosedLoopSlot.kSlot1);
    config.getInnerConfig()
      .smartCurrentLimit(80, 80);
    m_leaderMotor = SparkConfigurer.configSparkFlex(config);
    m_leaderEncoder = m_leaderMotor.getEncoder();

    SparkConfiguration followerConfig = SparkConstants.getShooterFlex(ShooterConstants.followerCanId, false,
        m_leaderMotor);
    m_followerMotor = SparkConfigurer.configSparkFlex(followerConfig);
    m_followerEncoder = m_followerMotor.getEncoder();

    m_motorController = m_leaderMotor.getClosedLoopController();
  }

  @Override
  public void setVelocity(double speed) {
    m_motorController.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot2, 0.0); // TODO:
    // max
    // motion
  }

  @Override
  public boolean isAtSetpoint() {
    return m_motorController.isAtSetpoint();
  }

  @Override
  public void setVoltage(double voltage) {
    m_leaderMotor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leaderVelocityMetersPerSecond = m_leaderEncoder.getVelocity() * 2 * Math.PI / 60; // rad/s
    inputs.leaderVelocityRPM = m_leaderEncoder.getVelocity(); // RPM
    inputs.leaderMotorVoltage = m_leaderMotor.getAppliedOutput() * m_leaderMotor.getBusVoltage(); // volts
    inputs.leaderMotorTemperatureCelsius = m_leaderMotor.getMotorTemperature(); // celsius
    inputs.leaderMotorCurrent = m_leaderMotor.getOutputCurrent(); // amps

    inputs.followerVelocityMetersPerSecond = m_followerEncoder.getVelocity() * 2 * Math.PI / 60; // rad/s
    inputs.followerVelocityRPM = m_followerEncoder.getVelocity(); // RPM
    inputs.followerMotorVoltage = m_followerMotor.getAppliedOutput() * m_followerMotor.getBusVoltage(); // volts
    inputs.followerMotorTemperatureCelsius = m_followerMotor.getMotorTemperature(); // celsius
    inputs.followerMotorCurrent = m_followerMotor.getOutputCurrent(); // amps

    inputs.averageVoltage = (inputs.leaderMotorVoltage + inputs.followerMotorVoltage) / 2.0; // rad/s
  }
}
