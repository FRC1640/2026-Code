package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.ShotControl.ShotSetpoint;

public interface ShooterIO extends AutoCloseable {

  @AutoLog
  public class ShooterIOInputs {
    public double leaderVelocityRadPerSec;
    public double leaderVelocityRPM;
    public double leaderMotorTemperatureCelsius;
    public double leaderMotorCurrent;
    public double leaderMotorVoltage;
    public double leaderMotorPositionRotations;

    public double followerVelocityRadPerSec;
    public double followerVelocityRPM;
    public double followerMotorCurrent;
    public double followerMotorVoltage;
    public double followerMotorTemperatureCelsius;
    public double followerMotorPositionRotations;

    public double averageVoltage;
  }

  public default void setVelocityRadPerSec(double velocityRadPerSec, double accelerationRadPerSecSquared) {
  }

  public default void setVelocity(ShotSetpoint setpoint) {
    setVelocityRadPerSec(setpoint.shooterVelocityRPM(), setpoint.shooterAccelerationRotationsPerMinuteSquared());
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
