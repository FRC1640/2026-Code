package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO extends AutoCloseable {
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
