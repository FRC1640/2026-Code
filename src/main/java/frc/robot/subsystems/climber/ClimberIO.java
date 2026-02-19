package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO extends AutoCloseable {
  @AutoLog
  public class ClimberIOInputs {
    public double encoderPosition;
    public double encoderVelocity;
    public double motorCurrent;
    public double motorVoltage;
    public double motorTemperature;
  }

  public default void setPosition(double position) {
  }

  public default void setVoltage(double voltage) {
  }

  public default void updateInputs(ClimberIOInputs inputs) {
  }

  @Override
  public default void close() {
  }
}
