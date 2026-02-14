package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends AutoCloseable {
  @AutoLog
  public static class IntakeIOInputs {
    public double motorVoltage;
    public double motorTemperature;
    public double motorCurrent;
    public double encoderVelocity;
    public double encoderPosition;
  }

  public default void updateInputs(IntakeIOInputs inputs) {
  }

  public default void setVoltage(double voltage) {
  }

  public default void setPosition(double pos) {
  }

  @Override
  public default void close() {
  }

}
