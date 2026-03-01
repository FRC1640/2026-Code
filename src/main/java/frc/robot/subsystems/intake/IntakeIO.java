package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends AutoCloseable {
  @AutoLog
  public static class IntakeIOInputs {
    public double motorVoltage;
    public double motorTemperatureCelsius;
    public double motorCurrent;
    public double positionRadians;
    public double velocityRadPerSec;
    public double positionDegrees;
    public double velocityDegreesPerSec;
    public double positionRawEncoderValue;
  }

  public default void updateInputs(IntakeIOInputs inputs) {
  }

  public default void setVoltage(double voltage) {
  }

  public default void setPosition(double pos) {
    setState(pos, 0);
  }

  public default void setState(double angleRadians, double angularVelocityRadPerSec) {
  }

  @Override
  public default void close() {
  }

}
