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

  public IntakeIOReal() {
    m_intakeMotor = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(IntakeConstants.canID, true));
    m_intakeEncoder = m_intakeMotor.getAbsoluteEncoder();
    m_intakePID = RobotPIDConstants.constructPID(RobotPIDConstants.intakeAngleReal);
    m_intakePID.enableContinuousInput(0, 0.999);
  }

  @Override
  public void setPosition(double pos, IntakeIOInputs inputs) {
    Logger.recordOutput("Subsystems/Intake/Setpoint", pos);
    setVoltage(IntakeConstants.intakePositionLimits.clampOutput(inputs.intakeEncoderPosition,
        VoltageLim.clampVoltage(m_intakePID.calculate(inputs.intakeEncoderPosition, pos))), inputs);
  }

  @Override
  public void setVoltage(double voltage, IntakeIOInputs inputs) {
    m_intakeMotor.setVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeMotorTemperature = m_intakeMotor.getMotorTemperature(); // degrees celsius
    inputs.intakeMotorCurrent = m_intakeMotor.getOutputCurrent(); // amps
    inputs.intakeMotorVoltage = m_intakeMotor.getAppliedOutput() * m_intakeMotor.getBusVoltage(); // volts
    inputs.intakeEncoderPosition = m_intakeEncoder.getPosition() * 2 * Math.PI; // radians
    inputs.intakeEncoderVelocity = m_intakeEncoder.getVelocity() * 2 * Math.PI / 60; // rad/s
  }
}
