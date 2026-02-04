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
  private SparkMax intakeMotor;
  private SparkMax intakeRoller;
  private SparkClosedLoopController intakeRollerPID;
  private PIDController intakePID = RobotPIDConstants.constructPID(RobotPIDConstants.intakeAngleReal);
  private AbsoluteEncoder intakeEncoder;
  private AbsoluteEncoder rollerEncoder;

  public IntakeIOReal() {
    SparkConfiguration config = SparkConstants.getDefaultMax(IntakeConstants.intakeCanID, true);
    config.getInnerConfig().closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
        RobotPIDConstants.intakeRollerReal.kP, RobotPIDConstants.intakeRollerReal.kI,
        RobotPIDConstants.intakeRollerReal.kD, ClosedLoopSlot.kSlot0);
    intakeMotor = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(IntakeConstants.intakeCanID, true));
    intakeRoller = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(IntakeConstants.rollerCanID, true));
    intakeEncoder = intakeMotor.getAbsoluteEncoder();
    rollerEncoder = intakeRoller.getAbsoluteEncoder();
    intakeRollerPID = intakeMotor.getClosedLoopController();
    intakePID.enableContinuousInput(0, 0.999);
  }

  @Override
  public void setIntakePosition(double pos, IntakeIOInputs inputs) {
    Logger.recordOutput("Subsystems/Intake/Setpoint", pos);
    setIntakeVoltage(IntakeConstants.intakePositionLimits.clampOutput(inputs.intakeEncoderPosition,
        VoltageLim.clampVoltage(intakePID.calculate(inputs.intakeEncoderPosition, pos))), inputs);
  }

  @Override
  public void setIntakeVoltage(double voltage, IntakeIOInputs inputs) {
    intakeMotor.setVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setRollerVelocity(double velocity, IntakeIOInputs inputs) {
    intakeRollerPID.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setRollerVoltage(double voltage, IntakeIOInputs inputs) {
    intakeRoller.setVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void runVoltages(double intakeVoltage, double rollerVoltage, IntakeIOInputs inputs) {
    setIntakeVoltage(intakeVoltage, inputs);
    setRollerVoltage(rollerVoltage, inputs);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeMotorTemperature = intakeMotor.getMotorTemperature(); // degrees celsius
    inputs.intakeMotorCurrent = intakeMotor.getOutputCurrent(); // amps
    inputs.intakeMotorVoltage = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage(); // volts
    inputs.intakeEncoderPosition = intakeEncoder.getPosition() * 2 * Math.PI; // radians
    inputs.intakeEncoderVelocity = intakeEncoder.getVelocity() * 2 * Math.PI / 60; // rad/s
    inputs.rollerMotorTemperature = intakeRoller.getMotorTemperature(); // degrees celsius
    inputs.rollerMotorCurrent = intakeRoller.getOutputCurrent(); // amps
    inputs.rollerMotorVoltage = intakeRoller.getAppliedOutput() * intakeMotor.getBusVoltage(); // volts
    inputs.rollerEncoderVelocity = rollerEncoder.getVelocity() * 2 * Math.PI / 60; // rad/s
  }
}
