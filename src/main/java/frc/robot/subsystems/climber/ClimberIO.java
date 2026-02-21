package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO extends AutoCloseable {
  @AutoLog
  public class ClimberIOInputs {
    public double positionMeters;
    public double velocityMetersPerSec;
    public double heightMeters;
    public double verticalVelocityMetersPerSec;
    public double motorCurrent;
    public double motorVoltage;
    public double motorTemperature;
  }

  public default void setPosition(double position) {
  }

  public default void setHeight(double height) {
  }

  public default void setVoltage(double voltage) {
  }

  public default void updateInputs(ClimberIOInputs inputs) {
  }

  @Override
  public default void close() {
  }
}
