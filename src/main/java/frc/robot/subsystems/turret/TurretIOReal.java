package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.disconnectMinMotorVelocity;
import static frc.robot.subsystems.turret.TurretConstants.disconnectMinPotVelocity;
import static frc.robot.subsystems.turret.TurretConstants.turretAngleLimits;
import static frc.robot.subsystems.turret.TurretConstants.velocityLimitRate;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.MotorLim;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class TurretIOReal implements TurretIO {
  private final SparkMax m_motor;
  private final SparkAnalogSensor m_encoder;
  private final RelativeEncoder m_relativeEncoder;
  private final ProfiledPIDController /* PIDController */ m_positionController;
  // private final SimpleMotorFeedforward m_feedforwardController;

  public TurretIOReal() {
    m_motor = SparkConfigurer.configSparkMax(TurretConstants.canId, SparkConstants.turretConfig);
    m_encoder = m_motor.getAnalog();
    m_positionController = RobotPIDConstants.constructProfiledPIDController(RobotPIDConstants.turretAnglePidReal,
        RobotPIDConstants.turretAngleConstraintsReal);
    // m_positionController =
    // RobotPIDConstants.constructPID(RobotPIDConstants.turretAnglePidReal);
    // m_feedforwardController =
    // RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.turretAngleFF);
    m_relativeEncoder = m_motor.getEncoder();
  }

  @Override
  public void setTurretState(double angle, double angularVelocity) {
    Logger.recordOutput("Subsystems/Turret/desiredAngle", angle);
    Logger.recordOutput("Subsystems/Turret/desiredVelocity", angularVelocity);

    double clampedAngle = TurretConstants.turretAngleLimits.clampPosition(angle);
    Logger.recordOutput("Subsystems/Turret/setpointAngle", clampedAngle);

    double clampedVelocity = angularVelocity;
    // limit velocity setpoint to slow down near limit
    double intervalPos = (angle - turretAngleLimits.low) / (turretAngleLimits.high - turretAngleLimits.low);
    double scaledVelocity = angularVelocity * trapezoidScale(intervalPos);
    Logger.recordOutput("Shot/velocitySetpointScale", scaledVelocity / angularVelocity);
    boolean approachingLimit = (intervalPos > 0.5) ? angularVelocity > 0 : angularVelocity < 0;
    if (approachingLimit) {
      clampedVelocity = scaledVelocity;
    } else if (turretAngleLimits.inRange(angle)) {
      clampedVelocity = angularVelocity;
    } else {
      clampedVelocity = 0;
    }
    Logger.recordOutput("Subsystems/Turret/scaledVelocity", clampedVelocity);
    clampedVelocity = MathUtil.clamp(clampedVelocity, -TurretConstants.maxVelocityRadPerSec,
        TurretConstants.maxVelocityRadPerSec);
    Logger.recordOutput("Subsystems/Turret/setpointVelocity", angle);

    double voltage = m_positionController.calculate(getTurretPosition(),
        new TrapezoidProfile.State(clampedAngle, clampedVelocity));
    setVoltage(voltage);
  }

  private double trapezoidScale(double x) {
    return (0 <= x && x <= 1 / velocityLimitRate)
        ? x * velocityLimitRate
        : (1 - (1 / velocityLimitRate) <= x && x <= 1) ? -velocityLimitRate * (x - 1) : 1;
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
    return (m_encoder.getPosition() - TurretConstants.potVoltage0) * TurretConstants.turretPotToRadiansConversion + TurretConstants.turretAngle0Radians;
  }

  private double getTurretVelocity() {
    return m_encoder.getVelocity() * TurretConstants.turretPotToRadiansConversion;
  }

  public boolean isSensorDisconnected() {
    return Math.abs(m_relativeEncoder.getVelocity()) > disconnectMinMotorVelocity
        && Math.abs(m_encoder.getVelocity()) < disconnectMinPotVelocity;
  }
}
