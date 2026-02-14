package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim m_motor;
  private final PIDController m_positionController = RobotPIDConstants.constructPID(RobotPIDConstants.intakeSim);

  public IntakeIOSim() {
    DCMotor gearbox = DCMotor.getNEO(1);
    m_motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.00019125, IntakeConstants.gearRatio),
        gearbox);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    m_motor.update(0.02);

    // TODO: unit conversions
    inputs.motorTemperature = 0; // degrees celsius
    inputs.motorCurrent = m_motor.getCurrentDrawAmps(); // amps
    inputs.motorVoltage = m_motor.getInputVoltage(); // volts
    inputs.encoderPosition = m_motor.getAngularPositionRad(); // radians
    inputs.encoderVelocity = m_motor.getAngularVelocityRadPerSec(); // rad/s
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setInputVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setPosition(double pos) {
    Logger.recordOutput("Subsystems/Intake/Setpoint", pos);
    setVoltage(IntakeConstants.positionLimits.clampOutput(m_motor.getAngularPositionRad(),
        VoltageLim.clampVoltage(m_positionController.calculate(m_motor.getAngularPositionRad(), pos))));
  }
}
