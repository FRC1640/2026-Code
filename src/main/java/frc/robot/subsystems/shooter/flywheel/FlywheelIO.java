package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;

public interface FlywheelIO extends AutoCloseable {

  @AutoLog
  public class FlywheelIOInputs {
    public double leaderVelocity;
    public double leaderMotorTemperature;
    public double leaderMotorCurrent;
    public double leaderMotorVoltage;

    public double followerVelocity;
    public double followerMotorCurrent;
    public double followerMotorVoltage;
    public double followerMotorTemperature;

    public double averageVoltage;
  }

  public default void setVelocity(double velocity) {
  }

  public default void setVelocity(TurretSetpoint setpoint) {
    setVelocity(setpoint.flywheelSpeed());
  }

  public default void setVoltage(double voltage) {
  }

  public default void setFlywheelVoltage(double voltage) {

  }

  public default void updateInputs(FlywheelIOInputs inputs) {
  }

  @Override
  public default void close() {
  }

}
