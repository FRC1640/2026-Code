package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;
import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class IntakeIOReal implements IntakeIO {
  private final SparkMax m_intakeMotor;
  private final AbsoluteEncoder m_intakeEncoder;
  private final PIDController m_intakePID;
  private final SparkMax m_rollerMotor;
  private final AbsoluteEncoder m_rollerEncoder;
  private final SparkClosedLoopController m_rollerPID;

  public IntakeIOReal() {
    m_intakeMotor = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(IntakeConstants.intakeCanID, true));
    m_intakeEncoder = m_intakeMotor.getAbsoluteEncoder();
    m_intakePID = RobotPIDConstants.constructPID(RobotPIDConstants.intakeAngleReal);
    m_intakePID.enableContinuousInput(0, 0.999);

    SparkConfiguration rollerConfig = SparkConstants.getDefaultMax(IntakeConstants.intakeCanID, true);
    rollerConfig.getInnerConfig().closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
        RobotPIDConstants.intakeRollerReal.kP, RobotPIDConstants.intakeRollerReal.kI,
        RobotPIDConstants.intakeRollerReal.kD, ClosedLoopSlot.kSlot0);

    m_rollerMotor = SparkConfigurer.configSparkMax(rollerConfig);
    m_rollerEncoder = m_rollerMotor.getAbsoluteEncoder();
    m_rollerPID = m_intakeMotor.getClosedLoopController();
  }

  @Override
  public void setIntakePosition(double pos, IntakeIOInputs inputs) {
    Logger.recordOutput("Subsystems/Intake/Setpoint", pos);
    setIntakeVoltage(IntakeConstants.intakePositionLimits.clampOutput(inputs.intakeEncoderPosition,
        VoltageLim.clampVoltage(m_intakePID.calculate(inputs.intakeEncoderPosition, pos))), inputs);
  }

  @Override
  public void setIntakeVoltage(double voltage, IntakeIOInputs inputs) {
    m_intakeMotor.setVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setRollerVelocity(double velocity, IntakeIOInputs inputs) {
    m_rollerPID.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setRollerVoltage(double voltage, IntakeIOInputs inputs) {
    m_rollerMotor.setVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeMotorTemperature = m_intakeMotor.getMotorTemperature(); // degrees celsius
    inputs.intakeMotorCurrent = m_intakeMotor.getOutputCurrent(); // amps
    inputs.intakeMotorVoltage = m_intakeMotor.getAppliedOutput() * m_intakeMotor.getBusVoltage(); // volts
    inputs.intakeEncoderPosition = m_intakeEncoder.getPosition() * 2 * Math.PI; // radians
    inputs.intakeEncoderVelocity = m_intakeEncoder.getVelocity() * 2 * Math.PI / 60; // rad/s
    inputs.rollerMotorTemperature = m_rollerMotor.getMotorTemperature(); // degrees celsius
    inputs.rollerMotorCurrent = m_rollerMotor.getOutputCurrent(); // amps
    inputs.rollerMotorVoltage = m_rollerMotor.getAppliedOutput() * m_intakeMotor.getBusVoltage(); // volts
    inputs.rollerEncoderVelocity = m_rollerEncoder.getVelocity() * 2 * Math.PI / 60; // rad/s
  }
}
