package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.ShotControl.TurretSetpoint;

public interface HoodIO extends AutoCloseable {
  @AutoLog
  public class HoodIOInputs {
    public double angleRadians;
    public double angularVelocityRadPerSec;
    public double angleDegrees;
    public double angularVelocityDegreesPerSec;
    public double motorCurrent;
    public double motorVoltage;
    public double motorTemperatureCelsius;
  }

  public default void setAngleRadians(double angle) {
  }

  public default void setAngle(TurretSetpoint setpoint) {
    setAngleRadians(Math.toRadians(setpoint.hoodAngleDeg()));
  }

  public default void setVoltage(double voltage) {
  }

  public default void updateInputs(HoodIOInputs inputs) {
  }

  @Override
  public default void close() {
  }

}
