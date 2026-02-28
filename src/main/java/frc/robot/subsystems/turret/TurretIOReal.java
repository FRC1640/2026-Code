package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.constants.RobotPIDConstants;
import static frc.robot.subsystems.turret.TurretConstants.disconnectMinMotorVelocity;
import static frc.robot.subsystems.turret.TurretConstants.disconnectMinPotVelocity;
import frc.robot.util.limits.MotorLim;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class TurretIOReal implements TurretIO {
  private final SparkMax m_motor;
  private final SparkAnalogSensor m_encoder;
  private final RelativeEncoder m_relativeEncoder;
  private final ProfiledPIDController m_positionController;
  // private final SimpleMotorFeedforward m_feedforwardController;

  public TurretIOReal() {
    m_motor = SparkConfigurer.configSparkMax(TurretConstants.canId, SparkConstants.turretConfig);
    m_encoder = m_motor.getAnalog();
    m_positionController = RobotPIDConstants.constructProfiledPIDController(RobotPIDConstants.turretAnglePidReal, RobotPIDConstants.turretAngleConstraintsReal);
    // m_feedforwardController = RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.turretAngleFF);
    m_relativeEncoder = m_motor.getEncoder();
  }

  @Override
  public void setTurretState(double angle, double angularVelocity) {
    double clampedAngle = TurretConstants.turretAngleLimits.clampPosition(angle);
    double voltage = m_positionController.calculate(getTurretPosition(), clampedAngle)
        ;// + m_feedforwardController.calculate(angularVelocity);
    setVoltage(voltage);
  }

  @Override
  public void setVoltage(double voltage) {
    double voltageClamped = MotorLim.clampVoltage(voltage);
    voltageClamped = TurretConstants.turretAngleLimits.clampOutput(getTurretPosition(), voltageClamped);
    m_motor.setVoltage(voltageClamped);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.angleRadians = getTurretPosition();
    inputs.angularVelocityRadPerSec = getTurretVelocity();
    inputs.angleDegrees = inputs.angleRadians * 180 / Math.PI;
    inputs.angularVelocityDegreesPerSec = inputs.angularVelocityRadPerSec * 180 / Math.PI;
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorVoltage = m_motor.getBusVoltage() * m_motor.getAppliedOutput();
    inputs.motorTemperatureCelsius = m_motor.getMotorTemperature();

    Logger.recordOutput("Subsystems/Turret/encoderPositionRawVolts", m_encoder.getPosition());
    Logger.recordOutput("Subsystems/Turret/encoderVelocityRawVoltsPerSecond", m_encoder.getVelocity());
  }

  private double getTurretPosition() {
    return (m_encoder.getPosition() - TurretConstants.pot0degVoltage)
        / (TurretConstants.pot90degVoltage - TurretConstants.pot0degVoltage) * Math.PI / 2;
  }

  private double getTurretVelocity() {
    return m_encoder.getVelocity() / (TurretConstants.pot90degVoltage - TurretConstants.pot0degVoltage) * Math.PI
        / 2;
  }

  public boolean isSensorDisconnected() {
    return Math.abs(m_relativeEncoder.getVelocity()) > disconnectMinMotorVelocity
        && Math.abs(m_encoder.getVelocity()) < disconnectMinPotVelocity;
  }
}
