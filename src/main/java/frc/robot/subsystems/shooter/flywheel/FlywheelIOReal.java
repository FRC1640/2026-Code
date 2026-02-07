package frc.robot.subsystems.shooter.flywheel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class FlywheelIOReal implements FlywheelIO {
  private final SparkFlex m_leaderMotor;
  private final RelativeEncoder m_leaderEncoder;
  private final SparkFlex m_followerMotor;
  private final RelativeEncoder m_followerEncoder;

  private final SparkClosedLoopController m_motorController;

  public FlywheelIOReal() {
    SparkConfiguration config = SparkConstants.getDefaultFlex(FlywheelConstants.canId, false);
    config.getInnerConfig().closedLoop.pid(0.0001, 0, 0, ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    m_leaderMotor = SparkConfigurer.configSparkFlex(config);

    SparkConfiguration followerConfig = SparkConstants.getDefaultFlex(FlywheelConstants.followerCanId, false,
        m_leaderMotor);
    m_followerMotor = SparkConfigurer.configSparkFlex(followerConfig);
    m_motorController = m_leaderMotor.getClosedLoopController();

    m_followerEncoder = m_followerMotor.getEncoder();
    m_leaderEncoder = m_leaderMotor.getEncoder();
  }

  @Override
  public void setVelocity(double speed) {
    m_motorController.setSetpoint(speed, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, 0.0); // TODO:
    // max
    // motion
  }

  @Override
  public void setVoltage(double voltage) {
    m_leaderMotor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.leaderVelocity = m_leaderEncoder.getVelocity() * 2 * Math.PI / 60; // rad/s 
    inputs.leaderMotorVoltage = m_leaderMotor.getAppliedOutput() * m_leaderMotor.getBusVoltage(); // volts
    inputs.leaderMotorTemperature = m_leaderMotor.getMotorTemperature(); // celsius
    inputs.leaderMotorCurrent = m_leaderMotor.getOutputCurrent(); // amps

    inputs.followerVelocity = m_followerEncoder.getVelocity()  * 2 * Math.PI / 60; // rad/s
    inputs.followerMotorVoltage = m_followerMotor.getAppliedOutput() * m_followerMotor.getBusVoltage(); // volts
    inputs.followerMotorTemperature = m_followerMotor.getMotorTemperature(); // celsius
    inputs.followerMotorCurrent = m_followerMotor.getOutputCurrent(); // amps

    inputs.averageVoltage = (inputs.leaderMotorVoltage + inputs.followerMotorVoltage) / 2.0; // rad/s
  }
}
