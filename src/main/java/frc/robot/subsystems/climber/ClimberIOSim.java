package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.limits.VoltageLim;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class ClimberIOSim implements ClimberIO {
  private final SparkFlex m_motor;
  private final SparkAbsoluteEncoder m_encoder;
  private final SparkClosedLoopController m_positionController;

  private final SparkFlexSim m_motorSim;
  private final SparkAbsoluteEncoderSim m_encoderSim;

  public ClimberIOSim() {
    DCMotor motorGearboxSim = DCMotor.getNeoVortex(1);
    m_motor = SparkConfigurer.configSparkFlex(ClimberConstants.canId, SparkConstants.climberConfig);
    m_encoder = m_motor.getAbsoluteEncoder();
    m_positionController = m_motor.getClosedLoopController();

    m_motorSim = new SparkFlexSim(m_motor, motorGearboxSim);
    m_encoderSim = new SparkAbsoluteEncoderSim(m_motor);
  }

  @Override
  public void setPosition(double position) {
    Logger.recordOutput("Subsystems/Climber/setpoint", position);
    double positionClamped = ClimberConstants.positionLimitsMeters.clampPosition(position);
    Logger.recordOutput("Subsystems/Climber/setpointClamped", positionClamped);
    m_positionController.setSetpoint(positionClamped, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVoltage(double voltage) {
    Logger.recordOutput("Subsystems/Climber/desiredVoltage", voltage);
    double voltageClamped = VoltageLim.clampVoltage(voltage);
    voltageClamped = ClimberConstants.positionLimitsMeters.clampOutput(m_encoder.getPosition(), voltageClamped);
    Logger.recordOutput("Subsystems/Climber/clampedVoltage", voltageClamped);
    m_motor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    m_motorSim.iterate(Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
        0), 12, // Simulated battery voltage, in Volts
        0.02);
    m_encoderSim.iterate(Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
        0), 12);
    inputs.positionMeters = m_encoder.getPosition();
    inputs.velocityMetersPerSec = m_encoder.getVelocity();
    inputs.heightMeters = ClimberConstants.climberRetractedHeight
        + inputs.positionMeters * Math.cos(ClimberConstants.climberAngleRadians);
    inputs.verticalVelocityMetersPerSec = inputs.velocityMetersPerSec
        * Math.cos(ClimberConstants.climberAngleRadians);
    inputs.motorCurrent = m_motor.getOutputCurrent();
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.motorTemperature = m_motor.getMotorTemperature();
  }
}
