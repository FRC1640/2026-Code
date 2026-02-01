package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO extends AutoCloseable {
  @AutoLog
  public class HopperIOInputs {
    public double hopperMotorCurrent;
    public double hopperMotorVoltage;
    public double hopperMotorTemperature;
  }

  public default void setHopperVoltage(double voltage) {
  }

  public default void updateInputs(HopperIOInputs inputs) {
  }

  @Override
  public default void close() {
  }
}
