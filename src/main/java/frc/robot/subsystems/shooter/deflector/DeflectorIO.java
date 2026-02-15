package frc.robot.subsystems.shooter.deflector;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;

public interface DeflectorIO extends AutoCloseable {
  @AutoLog
  public class DeflectorIOInputs {
    public double angleRadians;
    public double motorTemperatureCelsius;
    public double motorCurrent;
    public double motorVoltage;
  }

  public default void setAngle(double angle) {
  }

  public default void setAngle(TurretSetpoint setpoint) {
    setAngle(setpoint.hoodAngle());
  }

  public default void updateInputs(DeflectorIOInputs inputs) {
  }

  public default void setVoltage(double voltage) {
  }

  @Override
  public default void close() {
  }

}
