package frc.robot.subsystems.rollers;

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

public class RollerIOReal implements RollerIO {
  private final SparkMax m_rollerMotor;
  private final AbsoluteEncoder m_rollerEncoder;
  private final SparkClosedLoopController m_rollerPID;

  public RollerIOReal() {

    SparkConfiguration rollerConfig = SparkConstants.getDefaultMax(RollerConstants.canID, true);
    rollerConfig.getInnerConfig().closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
        RobotPIDConstants.intakeRollerReal.kP, RobotPIDConstants.intakeRollerReal.kI,
        RobotPIDConstants.intakeRollerReal.kD, ClosedLoopSlot.kSlot0);

    m_rollerMotor = SparkConfigurer.configSparkMax(rollerConfig);
    m_rollerEncoder = m_rollerMotor.getAbsoluteEncoder();
    m_rollerPID = m_rollerMotor.getClosedLoopController();
  }

  @Override
  public void setVelocity(double velocity, RollerIOInputs inputs) {
    m_rollerPID.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVoltage(double voltage, RollerIOInputs inputs) {
    m_rollerMotor.setVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.rollerMotorTemperature = m_rollerMotor.getMotorTemperature(); // degrees celsius
    inputs.rollerMotorCurrent = m_rollerMotor.getOutputCurrent(); // amps
    inputs.rollerMotorVoltage = m_rollerMotor.getAppliedOutput() * m_rollerMotor.getBusVoltage(); // volts
    inputs.rollerEncoderVelocity = m_rollerEncoder.getVelocity() * 2 * Math.PI / 60; // rad/s
  }
}
