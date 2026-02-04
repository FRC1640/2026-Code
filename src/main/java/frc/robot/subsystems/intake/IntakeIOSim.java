package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim intakeMotor;
  private DCMotorSim intakeRoller;
  private PIDController intakePID = RobotPIDConstants.constructPID(RobotPIDConstants.intakeAngleSim);
  private PIDController intakeRollerPID = RobotPIDConstants.constructPID(RobotPIDConstants.intakeRollerSim);

  public IntakeIOSim() {
    DCMotor intakeGearbox = DCMotor.getNEO(1);
    intakeMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(intakeGearbox, 0.00019125, IntakeConstants.gearRatio),
        intakeGearbox);
    DCMotor rollerGearbox = DCMotor.getNEO(1);
    intakeRoller = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(rollerGearbox, 0.00019125, IntakeConstants.rollerGearRatio),
        rollerGearbox);
  }
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotor.update(0.02);

    // TODO: unit conversions
    inputs.intakeMotorTemperature = 0; // degrees celsius
    inputs.intakeMotorCurrent = intakeMotor.getCurrentDrawAmps(); // amps
    inputs.intakeMotorVoltage = intakeMotor.getInputVoltage(); // volts
    inputs.intakeEncoderPosition = intakeMotor.getAngularPositionRad(); // radians
    inputs.intakeEncoderVelocity = intakeMotor.getAngularVelocityRadPerSec(); // rad/s

    inputs.rollerMotorTemperature = 0; // degrees celsius
    inputs.rollerMotorCurrent = intakeRoller.getCurrentDrawAmps(); // amps
    inputs.rollerMotorVoltage = intakeRoller.getInputVoltage(); // volts
    inputs.rollerEncoderVelocity = intakeRoller.getAngularVelocityRadPerSec(); // rad/s
  }

  @Override
  public void setIntakeVoltage(double voltage, IntakeIOInputs inputs) {
    intakeMotor.setInputVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setRollerVoltage(double voltage, IntakeIOInputs inputs) {
    intakeRoller.setInputVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setRollerVelocity(double velocity, IntakeIOInputs inputs) {
    setRollerVoltage(
        IntakeConstants.intakePositionLimits.clampOutput(inputs.rollerEncoderVelocity,
            VoltageLim.clampVoltage(intakeRollerPID.calculate(inputs.rollerEncoderVelocity, velocity))),
        inputs);
  }

  @Override
  public void setIntakePosition(double pos, IntakeIOInputs inputs) {
    Logger.recordOutput("Subsystems/Intake/Setpoint", pos);
    setIntakeVoltage(IntakeConstants.intakePositionLimits.clampOutput(inputs.intakeEncoderPosition,
        VoltageLim.clampVoltage(intakePID.calculate(inputs.intakeEncoderPosition, pos))), inputs);
  }

  @Override
  public void runVoltages(double intakeVoltage, double rollerVoltage, IntakeIOInputs inputs) {
    setIntakeVoltage(intakeVoltage, inputs);
    setRollerVoltage(rollerVoltage, inputs);
  }
}
