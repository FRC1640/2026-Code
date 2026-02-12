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
  private PIDController intakePID = RobotPIDConstants.constructPID(RobotPIDConstants.intakeAngleSim);

  public IntakeIOSim() {
    DCMotor intakeGearbox = DCMotor.getNEO(1);
    intakeMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(intakeGearbox, 0.00019125, IntakeConstants.gearRatio),
        intakeGearbox);
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
  }

  @Override
  public void setVoltage(double voltage, IntakeIOInputs inputs) {
    intakeMotor.setInputVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setPosition(double pos, IntakeIOInputs inputs) {
    Logger.recordOutput("Subsystems/Intake/Setpoint", pos);
    setVoltage(IntakeConstants.intakePositionLimits.clampOutput(inputs.intakeEncoderPosition,
        VoltageLim.clampVoltage(intakePID.calculate(inputs.intakeEncoderPosition, pos))), inputs);
  }
}
