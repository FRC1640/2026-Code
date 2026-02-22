package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.ShotControl.TurretSetpoint;

public interface ShooterIO extends AutoCloseable {

  @AutoLog
  public class ShooterIOInputs {
    public double leaderVelocityRadPerSec;
    public double leaderVelocityRPM;
    public double leaderMotorTemperatureCelsius;
    public double leaderMotorCurrent;
    public double leaderMotorVoltage;

    public double followerVelocityRadPerSec;
    public double followerVelocityRPM;
    public double followerMotorCurrent;
    public double followerMotorVoltage;
    public double followerMotorTemperatureCelsius;

    public double averageVoltage;
  }

  public default void setVelocityRadPerSec(double velocityRadPerSec) {
  }

  public default void setVelocity(TurretSetpoint setpoint) {
    setVelocityRadPerSec(setpoint.shooterVelocityRPM());
  }

  public default void setVoltage(double voltage) {

  }

  public default boolean isAtSetpoint() {
    return false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {
  }

  @Override
  public default void close() {
  }

}
