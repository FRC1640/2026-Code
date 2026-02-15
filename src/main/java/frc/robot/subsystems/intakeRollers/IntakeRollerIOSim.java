package frc.robot.subsystems.intakeRollers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private final DCMotorSim m_motor;
  private final PIDController m_velocityController = RobotPIDConstants.constructPID(RobotPIDConstants.rollerSim);

  public IntakeRollerIOSim() {
    DCMotor rollerGearbox = DCMotor.getNEO(1);
    m_motor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(rollerGearbox, 0.00019125, IntakeRollerConstants.gearRatio),
        rollerGearbox);
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setInputVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setVelocity(double velocity) {
    setVoltage(m_velocityController.calculate(m_motor.getAngularVelocityRadPerSec(), velocity));
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.motorTemperatureCelsius = 0; // degrees celsius
    inputs.motorCurrent = m_motor.getCurrentDrawAmps(); // amps
    inputs.motorVoltage = m_motor.getInputVoltage(); // volts
    inputs.encoderVelocityRadiansPerSecond = m_motor.getAngularVelocityRadPerSec(); // rad/s
  }
}
