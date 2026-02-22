package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.ShotControl.TurretSetpoint;

public interface HoodIO extends AutoCloseable {
  @AutoLog
  public class HoodIOInputs {
    public double angleHorizontalRadians;
    public double angleVerticalRadians;
    /** Rate of change of radian angle with HORIZONTAL with respect to time. */
    public double angularVelocityRadPerSec;
    public double angleHorizontalDegrees;
    public double angleVerticalDegrees;
    /** Rate of change of degree angle with HORIZONTAL with respect to time. */
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
