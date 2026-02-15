package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.ShotControl.TurretSetpoint;

public interface TurretIO extends AutoCloseable {

  @AutoLog
  public class TurretIOInputs {
    public double angleRadians;
    public double angularVelocityMetersPerSecond;
    public double motorCurrent;
    public double motorVoltage;
    public double motorTemperatureCelsius;
  }

  public default void setTurretState(TurretSetpoint setpoint) {
    setTurretState(setpoint.turretAngleRad(), setpoint.turretOmegaRadPerSec());
  }

  public default void setTurretState(double angle, double angularVelocity) {
  }

  public default void setVoltage(double voltage) {
  }

  public default boolean isSensorDisconnected() {
    return false;
  }

  public default void updateInputs(TurretIOInputs inputs) {
  }

  @Override
  public default void close() {
  }

}
