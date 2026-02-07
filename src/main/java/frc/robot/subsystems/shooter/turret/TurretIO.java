package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;

public interface TurretIO extends AutoCloseable {

  @AutoLog
  public class TurretIOInputs {
    public double angle;
    public double angularVelocity;
    public double motorCurrent;
    public double motorVoltage;
    public double motorTemperature;
  }

  public default void setTurretState(TurretSetpoint setpoint) {
    setTurretState(setpoint.turretAngle(), setpoint.turretOmega());
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
