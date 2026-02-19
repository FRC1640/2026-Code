package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.disconnectMinMotorVelocity;
import static frc.robot.subsystems.turret.TurretConstants.disconnectMinPotVelocity;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.MotorLim;
import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class TurretIOReal implements TurretIO {
  private final SparkMax m_motor;
  private final SparkAnalogSensor m_encoder;
  private final RelativeEncoder m_relativeEncoder;
  private final PIDController m_positionController;
  private final SimpleMotorFeedforward m_feedforwardController;

  public TurretIOReal() {
    SparkConfiguration config = SparkConstants.getDefaultMax(TurretConstants.canId, true);
    m_motor = SparkConfigurer.configSparkMax(config);
    m_encoder = m_motor.getAnalog();
    m_positionController = RobotPIDConstants.constructPID(RobotPIDConstants.turretAnglePidReal);
    m_feedforwardController = RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.turretAngleFF);
    m_relativeEncoder = m_motor.getEncoder();
  }

  @Override
  public void setTurretState(double angle, double angularVelocity) {
    double clampedAngle = TurretConstants.turretAngleLimits.clampPosition(angle);
    double voltage = m_positionController.calculate(getTurretPosition(), clampedAngle)
        + m_feedforwardController.calculate(angularVelocity);
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
    inputs.angularVelocityMetersPerSecond = getTurretVelocity();
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorVoltage = m_motor.getBusVoltage() * m_motor.getAppliedOutput();
    inputs.motorTemperatureCelsius = m_motor.getMotorTemperature();
  }

  private double getTurretPosition() {
    return (m_encoder.getPosition() - TurretConstants.potLowerVoltage)
        / (TurretConstants.potUpperVoltage - TurretConstants.potLowerVoltage) * 2 * Math.PI - Math.PI;
  }

  private double getTurretVelocity() {
    return m_encoder.getVelocity() / (TurretConstants.potUpperVoltage - TurretConstants.potLowerVoltage) * 2
        * Math.PI;
  }

  public boolean isSensorDisconnected() {
    return Math.abs(m_relativeEncoder.getVelocity()) > disconnectMinMotorVelocity
        && Math.abs(m_encoder.getVelocity()) < disconnectMinPotVelocity;
  }
}
