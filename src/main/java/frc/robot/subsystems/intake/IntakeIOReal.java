package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.util.limits.VoltageLim;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class IntakeIOReal implements IntakeIO {
  private SparkMax intakeMotor;
  private SparkMax intakeRoller;
  private PIDController intakePID = RobotPIDConstants.constructPID(RobotPIDConstants.intakeReal);
  private AbsoluteEncoder encoder;

  public IntakeIOReal() {
    intakeMotor = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(IntakeConstants.canID, true));
    intakeRoller = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(IntakeConstants.rollerCanID, true));
    encoder = intakeMotor.getAbsoluteEncoder();
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
  }

  @Override
  public void setMotorVoltage(double voltage, IntakeIOInputs inputs) {
    intakeMotor.setVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setRollerMotorVoltage(double voltage, IntakeIOInputs inputs){
    intakeRoller.setVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setMotorPosition(double pos, IntakeIOInputs inputs) {
    Logger.recordOutput("Subsystems/Intake/Setpoint", pos);
    setMotorVoltage(IntakeConstants.intakePositionLimits.clampOutput(inputs.encoderPosition,
        VoltageLim.clampVoltage(intakePID.calculate(inputs.encoderPosition, pos))), inputs);
  }
}
