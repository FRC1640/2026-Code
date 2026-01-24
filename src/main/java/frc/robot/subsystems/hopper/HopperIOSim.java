package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.limits.MotorLim;

public class HopperIOSim implements HopperIO {
  private DCMotorSim hopperMotorSim;

  public HopperIOSim() {
    DCMotor simGearbox = DCMotor.getNEO(1);
    hopperMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        simGearbox, 0.0002, 1),
      simGearbox);
  }

  @Override
  public void setHopperVoltage(double voltage) {
    hopperMotorSim.setInputVoltage(MotorLim.clampVoltage(voltage));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    hopperMotorSim.update(0.02);

    inputs.hopperMotorCurrent = hopperMotorSim.getCurrentDrawAmps();
    inputs.hopperMotorVoltage = hopperMotorSim.getInputVoltage();
    inputs.hopperMotorTemperature = 0;
  }
}
