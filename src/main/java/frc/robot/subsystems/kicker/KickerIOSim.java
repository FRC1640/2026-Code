package frc.robot.subsystems.kicker;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.MotorLim;

public class KickerIOSim implements KickerIO {
  private final DCMotorSim m_motorSim;
  private final PIDController m_velocityController;

  public KickerIOSim() {
    DCMotor simGearbox = DCMotor.getNEO(1);
    m_motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(simGearbox, 0.0002, 1), simGearbox);
    m_velocityController = RobotPIDConstants.constructPID(RobotPIDConstants.kickerVelocityPidSim);
  }

  @Override
  public void setVelocity(double velocity) {
    setVoltage(m_velocityController.calculate(m_motorSim.getAngularVelocityRadPerSec(), velocity));
  }

  @Override
  public void setVoltage(double voltage) {
    m_motorSim.setInputVoltage(MotorLim.clampVoltage(voltage));
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    m_motorSim.update(0.02);

    inputs.motorCurrent = m_motorSim.getCurrentDrawAmps();
    inputs.motorVoltage = m_motorSim.getInputVoltage();
    inputs.motorTemperatureCelsius = 0;
    inputs.motorVelocityRadPerSec = m_motorSim.getAngularVelocityRadPerSec();
    inputs.motorVelocityRPM = m_motorSim.getAngularVelocityRPM();
    inputs.motorTotalEnergy = inputs.motorCurrent * inputs.motorVoltage * 0.02;
    inputs.motorPower = inputs.motorVoltage * inputs.motorCurrent;

  }
}
