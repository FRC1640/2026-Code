package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO extends AutoCloseable {
  @AutoLog
  public class KickerIOInputs {
    public double motorCurrent;
    public double motorVoltage;
    public double motorTemperatureCelsius;
    public double motorVelocityRadPerSec;
    public double motorVelocityRPM;
    public double motorDrawJoules = 0;
    public double motorWattage;
  }

  public default void setVelocity(double velocity) {
  }

  public default void setVoltage(double voltage) {
  }

  public default void updateInputs(KickerIOInputs inputs) {
  }

  @Override
  public default void close() {
  }
}
