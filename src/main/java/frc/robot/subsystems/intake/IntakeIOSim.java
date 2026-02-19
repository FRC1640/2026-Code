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
    m_motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.00019125, 1), gearbox);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    m_motor.update(0.02);

    inputs.motorTemperatureCelsius = 0; // degrees celsius
    inputs.motorCurrent = m_motor.getCurrentDrawAmps(); // amps
    inputs.motorVoltage = m_motor.getInputVoltage(); // volts
    inputs.positionRadians = m_motor.getAngularPositionRad(); // radians
    inputs.velocityRadPerSec = m_motor.getAngularVelocityRadPerSec(); // rad/s
    inputs.positionDegrees = inputs.positionRadians * 180 / Math.PI;
    inputs.velocityDegreesPerSec = inputs.positionRadians * 180 / Math.PI;
  }

  @Override
  public void setVoltage(double voltage) {
    double voltageClamped = VoltageLim.clampVoltage(voltage);
    voltageClamped = IntakeConstants.positionLimits.clampOutput(m_motor.getAngularPositionRad(), voltageClamped);
    m_motor.setInputVoltage(voltageClamped);
  }

  @Override
  public void setPosition(double pos) {
    Logger.recordOutput("Subsystems/Intake/Setpoint", pos);
    setVoltage(IntakeConstants.positionLimits.clampOutput(m_motor.getAngularPositionRad(),
        VoltageLim.clampVoltage(m_positionController.calculate(m_motor.getAngularPositionRad(), pos))));
  }
}
