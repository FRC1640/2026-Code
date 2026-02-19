package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class IntakeIOReal implements IntakeIO {
  private final SparkMax m_motor;
  private final AbsoluteEncoder m_encoder;
  private final PIDController m_positionController;

  public IntakeIOReal() {
    m_motor = SparkConfigurer.configSparkMax(IntakeConstants.canId, SparkConstants.intakeConfig);
    m_encoder = m_motor.getAbsoluteEncoder();
    m_positionController = RobotPIDConstants.constructPID(RobotPIDConstants.intakeReal);
  }

  @Override
  public void setPosition(double positionRadians) {
    Logger.recordOutput("Subsystems/Intake/setpointRadians", positionRadians);
    Logger.recordOutput("Subsystems/Intake/setpointDegrees", positionRadians * 180 / Math.PI);
    double voltage = m_positionController.calculate(
        m_encoder.getPosition() * 2 * Math.PI + IntakeConstants.intakeZeroOffsetRadians, positionRadians);
    setVoltage(voltage);
  }

  @Override
  public void setVoltage(double voltage) {
    double voltageClamped = VoltageLim.clampVoltage(voltage);
    voltageClamped = IntakeConstants.positionLimits.clampOutput(
        m_encoder.getPosition() * 2 * Math.PI + IntakeConstants.intakeZeroOffsetRadians, voltageClamped);
    Logger.recordOutput("Subsystems/Intake/setpointVoltage", voltageClamped);
    m_motor.setVoltage(voltageClamped);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorTemperatureCelsius = m_motor.getMotorTemperature(); // degrees celsius
    inputs.motorCurrent = m_motor.getOutputCurrent(); // amps
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage(); // volts
    inputs.positionRadians = m_encoder.getPosition() * 2 * Math.PI + IntakeConstants.intakeZeroOffsetRadians; // radians
    inputs.velocityRadPerSec = m_encoder.getVelocity() * 2 * Math.PI / 60; // rad/s
    inputs.positionDegrees = inputs.positionRadians * 180 / Math.PI; // degrees
    inputs.velocityDegreesPerSec = inputs.velocityRadPerSec * 180 / Math.PI; // deg/s
  }
}
