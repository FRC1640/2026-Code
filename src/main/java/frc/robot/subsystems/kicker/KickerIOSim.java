package frc.robot.subsystems.kicker;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.limits.MotorLim;

public class KickerIOSim implements KickerIO {
  private final DCMotorSim m_motorSim;

  public KickerIOSim() {
    DCMotor simGearbox = DCMotor.getNEO(1);
    m_motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(simGearbox, 0.0002, 1), simGearbox);
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
    inputs.motorTemperature = 0;
    inputs.encoderPosition = m_motorSim.getAngularPositionRad(); // radians
    inputs.encoderVelocity = m_motorSim.getAngularVelocityRadPerSec(); // rad/s
  }
}
