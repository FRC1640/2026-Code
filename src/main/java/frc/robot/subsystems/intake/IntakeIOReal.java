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
  private PIDController intakePID = RobotPIDConstants.constructPID(RobotPIDConstants.intakeReal);
  private AbsoluteEncoder encoder;

  public IntakeIOReal() {
    intakeMotor = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(IntakeConstants.canID, true));
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
  }

  @Override
  public void setMotorVoltage(double voltage, IntakeIOInputs inputs) {
    intakeMotor.setVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setMotorPosition(double pos, IntakeIOInputs inputs) {
    Logger.recordOutput("Subsystems/Intake/Setpoint", pos);
    setMotorVoltage(VoltageLim.applyLimits(inputs.encoderPosition, intakePID.calculate(inputs.encoderPosition, pos),
        IntakeConstants.intakeLowerLimit, IntakeConstants.intakeUpperLimit), inputs);
  }
}
