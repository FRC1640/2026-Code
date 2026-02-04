package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO extends AutoCloseable {
  @AutoLog
  public class HopperIOInputs {
    public double motorCurrent;
    public double motorVoltage;
    public double motorTemperature;
  }

  public default void setVoltage(double voltage) {
  }

  public default void updateInputs(HopperIOInputs inputs) {
  }

  @Override
  public default void close() {
  }
}
