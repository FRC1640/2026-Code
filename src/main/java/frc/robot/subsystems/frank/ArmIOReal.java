package frc.robot.subsystems.frank;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.util.limits.MotorLim;
import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;
import frc.robot.util.spark.SparkPIDConstants;

public class ArmIOReal implements ArmIO {
  private SparkMax motor;
  private RelativeEncoder encoder;
  private SparkClosedLoopController pid;

  public ArmIOReal() {
    SparkConfiguration config =
      SparkConstants.getDefaultMax(11, false)
        .applyPIDConfig(new SparkPIDConstants(0.1, 0, 0, 60, 0, ClosedLoopSlot.kSlot0));
    motor = SparkConfigurer.configSparkMax(config);
    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();
  }

  @Override
  public void setMotorVoltage(double voltage) {
    voltage = MotorLim.clampVoltage(voltage);
    voltage = MotorLim.applyLimits(
      encoder.getPosition(),
      voltage,
      ArmConstants.lowerLimit,
      ArmConstants.upperLimit);
    motor.setVoltage(voltage);
  }

  @Override
  public void setMotorPosition(double pos) {
    if (MotorLim.applyLimits(
        encoder.getPosition(),
        1,
        ArmConstants.lowerLimit,
        ArmConstants.upperLimit) == 0) {
      pid.setSetpoint(0, ControlType.kVelocity);
    }
    pid.setSetpoint(pos, ControlType.kPosition);
  }

  @Override
  public void resetEncoder() {
    encoder.setPosition(0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.velocity = encoder.getVelocity();
    inputs.current = motor.getOutputCurrent();
    inputs.temperature = motor.getMotorTemperature();
  }
}
