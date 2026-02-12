package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class RollerIOSim implements RollerIO {
  private DCMotorSim intakeRoller;
  private PIDController intakeRollerPID = RobotPIDConstants.constructPID(RobotPIDConstants.intakeRollerSim);

  public RollerIOSim() {
    DCMotor rollerGearbox = DCMotor.getNEO(1);
    intakeRoller = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(rollerGearbox, 0.00019125, RollerConstants.gearRatio),
        rollerGearbox);
  }
  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.rollerMotorTemperature = 0; // degrees celsius
    inputs.rollerMotorCurrent = intakeRoller.getCurrentDrawAmps(); // amps
    inputs.rollerMotorVoltage = intakeRoller.getInputVoltage(); // volts
    inputs.rollerEncoderVelocity = intakeRoller.getAngularVelocityRadPerSec(); // rad/s
  }

  @Override
  public void setVoltage(double voltage, RollerIOInputs inputs) {
    intakeRoller.setInputVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setVelocity(double velocity, RollerIOInputs inputs) {
    setVoltage(
        RollerConstants.intakePositionLimits.clampOutput(inputs.rollerEncoderVelocity,
            VoltageLim.clampVoltage(intakeRollerPID.calculate(inputs.rollerEncoderVelocity, velocity))),
        inputs);
  }
}
