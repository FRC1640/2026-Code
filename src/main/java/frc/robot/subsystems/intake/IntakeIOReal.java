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
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.util.limits.VoltageLim;
import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class IntakeIOReal implements IntakeIO {
  private SparkMax intakeMotor;
  private SparkMax intakeRoller;
  private SparkClosedLoopController intakeRollerPID;
  private PIDController intakePID = RobotPIDConstants.constructPID(RobotPIDConstants.intakeAngleReal);
  private AbsoluteEncoder encoder;
  private AbsoluteEncoder rollerEncoder;

  public IntakeIOReal() {
    SparkConfiguration config = SparkConstants.getDefaultMax(IntakeConstants.canID, true);
    config.getInnerConfig().closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
        RobotPIDConstants.intakeRollerReal.kP, RobotPIDConstants.intakeRollerReal.kI,
        RobotPIDConstants.intakeRollerReal.kD, ClosedLoopSlot.kSlot0);
    config.getInnerConfig().closedLoop.feedForward.kV(0);
    intakeMotor = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(IntakeConstants.canID, true));
    intakeRoller = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(IntakeConstants.rollerCanID, true));
    encoder = intakeMotor.getAbsoluteEncoder();
    rollerEncoder = intakeRoller.getAbsoluteEncoder();
    intakeRollerPID = intakeMotor.getClosedLoopController();
    intakePID.enableContinuousInput(0, 0.999);

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorTemperature = intakeMotor.getMotorTemperature();
    inputs.motorCurrent = intakeMotor.getOutputCurrent();
    inputs.motorVoltage = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.encoderPosition = encoder.getPosition();
    inputs.encoderVelocity = encoder.getVelocity();
    inputs.rollerMotorTemperature = intakeRoller.getMotorTemperature();
    inputs.rollerMotorCurrent = intakeRoller.getOutputCurrent();
    inputs.rollerMotorVoltage = intakeRoller.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.rollerEncoderPosition = rollerEncoder.getPosition();
    inputs.rollerEncoderVelocity = rollerEncoder.getVelocity();
  }

  @Override
  public void setMotorVoltage(double voltage, IntakeIOInputs inputs) {
    intakeMotor.setVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setRollerMotorVoltage(double voltage, IntakeIOInputs inputs) {
    intakeRoller.setVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setRollerVelocity(double velocity, IntakeIOInputs inputs) {
    intakeRollerPID.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setMotorPosition(double pos, IntakeIOInputs inputs) {
    Logger.recordOutput("Subsystems/Intake/Setpoint", pos);
    setMotorVoltage(IntakeConstants.intakePositionLimits.clampOutput(inputs.encoderPosition,
        VoltageLim.clampVoltage(intakePID.calculate(inputs.encoderPosition, pos))), inputs);
  }
}
