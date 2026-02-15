package frc.robot.subsystems.intakeRollers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;
import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class IntakeRollerIOReal implements IntakeRollerIO {
  private final SparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_velocityController;

  public IntakeRollerIOReal() {

    SparkConfiguration config = SparkConstants.getDefaultMax(IntakeRollerConstants.canID, true);
    config.getInnerConfig().closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
        RobotPIDConstants.rollerReal.kP, RobotPIDConstants.rollerReal.kI, RobotPIDConstants.rollerReal.kD,
        ClosedLoopSlot.kSlot0);

    m_motor = SparkConfigurer.configSparkMax(config);
    m_encoder = m_motor.getEncoder();
    m_velocityController = m_motor.getClosedLoopController();
  }

  @Override
  public void setVelocity(double velocity) {
    m_velocityController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.motorTemperatureCelsius = m_motor.getMotorTemperature(); // degrees celsius
    inputs.motorCurrent = m_motor.getOutputCurrent(); // amps
    inputs.motorVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage(); // volts
    inputs.encoderVelocityRadiansPerSecond = m_encoder.getVelocity() * 2 * Math.PI / 60; // rad/s
  }
}
