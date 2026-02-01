package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;

public interface TurretIO extends AutoCloseable {

  @AutoLog
  public class TurretIOInputs {

    public double turretAngle;
    public double turretAngularVelocity;
    public double turretMotorCurrent;
    public double turretMotorVoltage;
    public double turretMotorTemperature;
  }

  public default void setTurretState(double angle, double angularVelocity) {
  }

  public default void setTurretState(TurretSetpoint setpoint) {
    setTurretState(setpoint.turretAngle(), setpoint.turretOmega());
  }

  public default void setTurretVoltage(double voltage) {
  }

  public default void updateInputs(TurretIOInputs inputs) {
  }

  @Override
  public default void close() {
  }

}
