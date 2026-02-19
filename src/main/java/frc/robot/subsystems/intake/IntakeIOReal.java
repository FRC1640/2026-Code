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
    m_motor = SparkConfigurer.configSparkMax(IntakeConstants.canID, SparkConstants.intakeConfig);
    m_encoder = m_motor.getAbsoluteEncoder();
    m_positionController = RobotPIDConstants.constructPID(RobotPIDConstants.intakeReal);
    m_positionController.enableContinuousInput(0, 0.999);
  }

  @Override
  public void setPosition(double pos) {
    Logger.recordOutput("Subsystems/Intake/Setpoint", pos);
    setVoltage(IntakeConstants.positionLimits.clampOutput(m_encoder.getPosition() * 2 * Math.PI,
        VoltageLim.clampVoltage(m_positionController.calculate(m_encoder.getPosition() * 2 * Math.PI, pos))));
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorTemperatureCelsius = m_motor.getMotorTemperature(); // degrees celsius
    inputs.motorCurrent = m_motor.getOutputCurrent(); // amps
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage(); // volts
    inputs.encoderPositionRadians = m_encoder.getPosition() * 2 * Math.PI; // radians
    inputs.encoderVelocityRadiansPerSecond = m_encoder.getVelocity() * 2 * Math.PI / 60; // rad/s
  }
}
