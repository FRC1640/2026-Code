package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class IntakeIOReal implements IntakeIO {
  private final SparkMax m_motor;
  private final AbsoluteEncoder m_encoder;
  private final ProfiledPIDController m_motionProfile;
  private final double kCos = 0.835;

  public IntakeIOReal() {
    m_motor = SparkConfigurer.configSparkMax(IntakeConstants.canId, SparkConstants.intakeConfig);
    m_encoder = m_motor.getAbsoluteEncoder();
    m_motionProfile = RobotPIDConstants.constructProfiledPIDController(RobotPIDConstants.intakeReal,
        RobotPIDConstants.intakeAngleConstraintsReal);
  }

  @Override
  public void setState(double angleRadians, double angularVelocityRadPerSec) {
    Logger.recordOutput("Subsystems/Intake/setpointRadians", angleRadians);
    Logger.recordOutput("Subsystems/Intake/setpointDegrees", angleRadians * 180 / Math.PI);
    Logger.recordOutput("Subsystems/Intake/setpointVelocityRadPerSec", angularVelocityRadPerSec);
    Logger.recordOutput("Subsystems/Intake/setpointVelocityDegreesPerSec",
        angularVelocityRadPerSec * 180 / Math.PI);
    double voltage = m_motionProfile.calculate(getPositionRadians(),
        new TrapezoidProfile.State(angleRadians, angularVelocityRadPerSec))
        + kCos * Math.cos(getPositionRadians() - Units.degreesToRadians(15));
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

  private double getPositionRadians() {
    return (m_encoder.getPosition() - IntakeConstants.intakeManualOffset)
        * IntakeConstants.intakeEncoderToRadiansConversion + IntakeConstants.intakeAngle0Radians;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorTemperatureCelsius = m_motor.getMotorTemperature(); // degrees celsius
    inputs.motorCurrent = m_motor.getOutputCurrent(); // amps
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage(); // volts
    inputs.positionRadians = getPositionRadians(); // radians
    inputs.velocityRadPerSec = m_encoder.getVelocity() * IntakeConstants.intakeEncoderToRadiansConversion; // rad/s
    inputs.positionDegrees = inputs.positionRadians * 180 / Math.PI; // degrees
    inputs.velocityDegreesPerSec = inputs.velocityRadPerSec * 180 / Math.PI; // deg/s
    inputs.positionRawEncoderValue = m_encoder.getPosition();
  }
}
