package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class IntakeIOReal implements IntakeIO {
  private final SparkMax m_motor;
  private final AbsoluteEncoder m_encoder;
  private final PIDController m_angleController;
  private final SimpleMotorFeedforward m_feedforwardController;
  private final PIDController m_velocityController;
  private final double kCos = 0.644;
  private final PIDController m_holdController;

  public IntakeIOReal() {
    m_motor = SparkConfigurer.configSparkMax(IntakeConstants.canId, SparkConstants.intakeConfig);
    m_encoder = m_motor.getAbsoluteEncoder();
    m_angleController = RobotPIDConstants.constructPID(RobotPIDConstants.intakeAnglePidReal);
    m_feedforwardController = RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.intakeFFReal);
    m_velocityController = RobotPIDConstants.constructPID(RobotPIDConstants.intakeVelocityPidReal);
    m_holdController = RobotPIDConstants.constructPID(RobotPIDConstants.intakeHoldPidReal);
  }

  @Override
  public void setState(double angleRadians, double angularVelocityRadPerSec) {
    Logger.recordOutput("Subsystems/Intake/setpointRadians", angleRadians);
    Logger.recordOutput("Subsystems/Intake/setpointDegrees", angleRadians * 180 / Math.PI);
    Logger.recordOutput("Subsystems/Intake/setpointVelocityRadPerSec", angularVelocityRadPerSec);
    Logger.recordOutput("Subsystems/Intake/setpointVelocityDegreesPerSec",
        angularVelocityRadPerSec * 180 / Math.PI);
    Logger.recordOutput("Subsystems/Intake/holding", false);
    double pidVoltage = m_angleController.calculate(getPositionRadians(), angleRadians);
    double ffVoltage = m_feedforwardController.calculate(angularVelocityRadPerSec);
    double kCosVoltage = kCos * Math.cos(getPositionRadians() - Units.degreesToRadians(15));
    Logger.recordOutput("Subsystems/Intake/pidVoltage", pidVoltage);
    Logger.recordOutput("Subsystems/Intake/ffVoltage", ffVoltage);
    Logger.recordOutput("Subsystems/Intake/kCosVoltage", kCosVoltage);
    setVoltage(pidVoltage + ffVoltage + kCosVoltage);
  }

  @Override
  public void setVelocity(double angularVelocityRadPerSec) {
    Logger.recordOutput("Subsystems/Intake/setpointVelocityRadPerSec", angularVelocityRadPerSec);
    Logger.recordOutput("Subsystems/Intake/setpointVelocityDegreesPerSec",
        angularVelocityRadPerSec * 180 / Math.PI);
    Logger.recordOutput("Subsystems/Intake/holding", false);
    double ffVoltage = m_feedforwardController.calculate(angularVelocityRadPerSec);
    double kCosVoltage = kCos * Math.cos(getPositionRadians() - Units.degreesToRadians(15));
    double pidVoltage = m_velocityController.calculate(getVelocityRadPerSec(), angularVelocityRadPerSec);
    Logger.recordOutput("Subsystems/Intake/velocityPidVoltage", pidVoltage);
    Logger.recordOutput("Subsystems/Intake/ffVoltage", ffVoltage);
    Logger.recordOutput("Subsystems/Intake/kCosVoltage", kCosVoltage);
    setVoltage(ffVoltage + kCosVoltage + pidVoltage);
  }

  @Override
  public void setPositionHold(double angleRadians) {
    Logger.recordOutput("Subsystems/Intake/holding", true);
    Logger.recordOutput("Subsystems/Intake/setpointRadians", angleRadians);
    Logger.recordOutput("Subsystems/Intake/setpointDegrees", angleRadians * 180 / Math.PI);
    Logger.recordOutput("Subsystems/Intake/setpointVelocityRadPerSec", 0.0);
    Logger.recordOutput("Subsystems/Intake/setpointVelocityDegreesPerSec", 0.0);
    double voltage;
    if (MathUtil.isNear(angleRadians, getPositionRadians(), Units.degreesToRadians(16))) {
      voltage = m_holdController.calculate(getPositionRadians(), angleRadians);
    } else {
      voltage = m_angleController.calculate(getPositionRadians(), angleRadians);
    }
    setVoltage(voltage);
  }

  @Override
  public void setVoltage(double voltage) {
    Logger.recordOutput("Subsystems/Intake/desiredVoltage", voltage);
    double voltageClamped = VoltageLim.clampVoltage(voltage);
    voltageClamped = IntakeConstants.positionLimitsRadians.clampOutput(getPositionRadians(), voltageClamped);
    Logger.recordOutput("Subsystems/Intake/setpointVoltage", voltageClamped);
    m_motor.setVoltage(voltageClamped);
  }

  @Override
  public double getPositionRadians() {
    return (m_encoder.getPosition() - IntakeConstants.intakeManualOffset)
        * IntakeConstants.intakeEncoderToRadiansConversion + IntakeConstants.intakeAngle0Radians;
  }

  private double getVelocityRadPerSec() {
    return m_encoder.getVelocity() * IntakeConstants.intakeEncoderToRadiansConversion;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorTemperatureCelsius = m_motor.getMotorTemperature(); // degrees celsius
    inputs.motorCurrent = m_motor.getOutputCurrent(); // amps
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage(); // volts
    inputs.positionRadians = getPositionRadians(); // radians
    inputs.velocityRadPerSec = getVelocityRadPerSec(); // rad/s
    inputs.positionDegrees = inputs.positionRadians * 180 / Math.PI; // degrees
    inputs.velocityDegreesPerSec = inputs.velocityRadPerSec * 180 / Math.PI; // deg/s

    Logger.recordOutput("Subsystems/Intake/encoderPositionRaw", m_encoder.getPosition());
  }
}
