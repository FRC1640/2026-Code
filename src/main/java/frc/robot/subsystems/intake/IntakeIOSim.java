package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.util.limits.VoltageLim;
import frc.robot.constants.RobotPIDConstants;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim intakeMotor;
  private DCMotorSim intakeRoller;
  private PIDController intakePID = RobotPIDConstants.constructPID(RobotPIDConstants.intakeAngleSim);
  private PIDController intakeRollerPID = RobotPIDConstants.constructPID(RobotPIDConstants.intakeRollerSim);

  public IntakeIOSim() {
    DCMotor simGearbox = DCMotor.getNEO(1);
    intakeMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(simGearbox, 0.00019125, IntakeConstants.gearRatio), simGearbox);
    intakeRoller = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(simGearbox, 0.00019125, IntakeConstants.rollerGearRatio),
        simGearbox);
  }
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotor.update(0.02);

    // TODO: unit conversions
    inputs.intakeMotorTemperature = 0;
    inputs.intakeMotorCurrent = intakeMotor.getCurrentDrawAmps();
    inputs.intakeMotorVoltage = intakeMotor.getInputVoltage();
    inputs.intakeEncoderPosition = intakeMotor.getAngularPositionRad();
    inputs.intakeEncoderVelocity = intakeMotor.getAngularVelocityRadPerSec();

    inputs.rollerMotorTemperature = 0;
    inputs.rollerMotorCurrent = intakeRoller.getCurrentDrawAmps();
    inputs.rollerMotorVoltage = intakeRoller.getInputVoltage();
    inputs.rollerEncoderVelocity = intakeRoller.getAngularVelocityRadPerSec();
  }

  @Override
  public void setMotorVoltage(double voltage, IntakeIOInputs inputs) {
    intakeMotor.setInputVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setRollerMotorVoltage(double voltage, IntakeIOInputs inputs) {
    intakeRoller.setInputVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setRollerVelocity(double velocity, IntakeIOInputs inputs) {
    setRollerMotorVoltage(
        IntakeConstants.intakePositionLimits.clampOutput(inputs.rollerEncoderVelocity,
            VoltageLim.clampVoltage(intakeRollerPID.calculate(inputs.rollerEncoderVelocity, velocity))),
        inputs);
  }

  @Override
  public void setMotorPosition(double pos, IntakeIOInputs inputs) {
    Logger.recordOutput("Subsystems/Intake/Setpoint", pos);
    setMotorVoltage(IntakeConstants.intakePositionLimits.clampOutput(inputs.intakeEncoderPosition,
        VoltageLim.clampVoltage(intakePID.calculate(inputs.intakeEncoderPosition, pos))), inputs);
  }
}
