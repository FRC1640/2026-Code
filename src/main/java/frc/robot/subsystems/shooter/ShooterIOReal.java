package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import frc.robot.util.limits.VoltageLim;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class ShooterIOReal implements ShooterIO {
  private final SparkFlex m_leaderMotor;
  private final RelativeEncoder m_leaderEncoder;
  private final SparkFlex m_followerMotor;
  private final RelativeEncoder m_followerEncoder;

  private final SparkClosedLoopController m_motorController;

  public ShooterIOReal() {
    m_leaderMotor = SparkConfigurer.configSparkFlex(ShooterConstants.canId, SparkConstants.shooterLeaderConfig);
    m_leaderEncoder = m_leaderMotor.getEncoder();

    m_followerMotor = SparkConfigurer.configSparkFlex(ShooterConstants.followerCanId,
        SparkConstants.shooterFollowerConfig);
    m_followerEncoder = m_followerMotor.getEncoder();

    m_motorController = m_leaderMotor.getClosedLoopController();
  }

  @Override
  public void setVelocityRadPerSec(double velocityRadPerSec) {
    Logger.recordOutput("Subsystems/Shooter/setpointVelocityRadPerSec", velocityRadPerSec);
    double velocityRPM = velocityRadPerSec * 60 / (2 * Math.PI);
    Logger.recordOutput("Subsystems/Shooter/setpointVelocityRPM", velocityRPM);
    m_motorController.setSetpoint(velocityRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot2);
  }

  @Override
  public boolean isAtSetpoint() {
    return m_motorController.isAtSetpoint();
  }

  @Override
  public void setVoltage(double voltage) {
    Logger.recordOutput("Subsystems/Shooter/desiredVoltage", voltage);
    double voltageClamped = VoltageLim.clampVoltage(voltage);
    Logger.recordOutput("Subsystems/Shooter/setpointVoltage", voltageClamped);
    m_leaderMotor.setVoltage(voltageClamped);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leaderVelocityRadPerSec = m_leaderEncoder.getVelocity() * 2 * Math.PI / 60; // rad/s
    inputs.leaderVelocityRPM = m_leaderEncoder.getVelocity(); // RPM
    inputs.leaderMotorVoltage = m_leaderMotor.getAppliedOutput() * m_leaderMotor.getBusVoltage(); // volts
    inputs.leaderMotorTemperatureCelsius = m_leaderMotor.getMotorTemperature(); // celsius
    inputs.leaderMotorCurrent = m_leaderMotor.getOutputCurrent(); // amps

    inputs.followerVelocityRadPerSec = m_followerEncoder.getVelocity() * 2 * Math.PI / 60; // rad/s
    inputs.followerVelocityRPM = m_followerEncoder.getVelocity(); // RPM
    inputs.followerMotorVoltage = m_followerMotor.getAppliedOutput() * m_followerMotor.getBusVoltage(); // volts
    inputs.followerMotorTemperatureCelsius = m_followerMotor.getMotorTemperature(); // celsius
    inputs.followerMotorCurrent = m_followerMotor.getOutputCurrent(); // amps

    inputs.averageVoltage = (inputs.leaderMotorVoltage + inputs.followerMotorVoltage) / 2.0; // rad/s
  }
}
