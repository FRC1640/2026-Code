package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO extends AutoCloseable {
  @AutoLog
  public class KickerIOInputs {
    public double motorCurrent;
    public double motorVoltage;
    public double motorTemperature;
    public double encoderVelocity;
    public double encoderPosition;
  }

  public default void setVoltage(double voltage) {
  }

  public default void updateInputs(KickerIOInputs inputs) {
  }

  @Override
  public default void close() {
  }
}
