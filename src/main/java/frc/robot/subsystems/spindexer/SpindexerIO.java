package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO extends AutoCloseable {
  @AutoLog
  public static class SpindexerIOInputs {
    public double motorVelocityRadPerSec = 0.0;
    public double motorVelocityRPM = 0.0;
    public double motorVoltage = 0.0;
    public double motorCurrent = 0.0;
    public double motorTemperatureCelsius = 0.0;
    public boolean isJammed = false;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {
  }

  public default void setVoltage(double voltage) {
  }

  @Override
  default void close() {
  }
}
