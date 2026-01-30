package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Robot;
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
  }

  public default void setVelocity(double velocity) {
  }

  public default void setVelocity(TurretSetpoint setpoint) {
    setVelocity(setpoint.flywheelSpeed());
  }

  public default void setVoltage(double voltage) {
  }

  public default double getAverageVoltage() {
    return Double.NaN;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {
  }

  @Override
  public default void close() {
  }

  public static FlywheelIO getIOByMode() {
    return switch (Robot.getMode()) {
      case REAL -> new FlywheelIOReal();
      case SIM -> new FlywheelIOSim();
      case REPLAY -> new FlywheelIO() {
      };
    };
  }

}
