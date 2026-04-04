package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.ShotControl.ShotSetpoint;

public interface TurretIO extends AutoCloseable {

  @AutoLog
  public class TurretIOInputs {
    public double angleRadians;
    public double angularVelocityRadPerSec;
    public double angleDegrees;
    public double angularVelocityDegreesPerSec;
    public double motorCurrent;
    public double motorVoltage;
    public double motorTemperatureCelsius;
    public double motorTotalDrawJoules = 0;
    public double motorWattage;
  }

  public default void setTurretState(ShotSetpoint setpoint) {
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
