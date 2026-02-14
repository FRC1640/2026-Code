package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.ShotControl.TurretSetpoint;

public interface HoodIO extends AutoCloseable {
  @AutoLog
  public class HoodIOInputs {
    public double angle;
    public double motorTemperature;
    public double motorCurrent;
    public double motorVoltage;
  }

  public default void setAngleRad(double angle) {
  }

  public default void setAngle(TurretSetpoint setpoint) {
    setAngleRad(Math.toRadians(setpoint.hoodAngleDeg()));
  }

  public default void updateInputs(HoodIOInputs inputs) {
  }

  public default void setVoltage(double voltage) {
  }

  @Override
  public default void close() {
  }

}
