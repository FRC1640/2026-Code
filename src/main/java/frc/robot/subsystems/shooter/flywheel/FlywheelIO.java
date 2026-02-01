package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;

public interface FlywheelIO extends AutoCloseable {
  @AutoLog
  public class FlywheelIOInputs {
    public double flywheelSpeed;
    public double flywheelMotorTemperature;
    public double flywheelMotorCurrent;
    public double flywheelMotorVoltage;

    public double flywheelFollowerSpeed;
    public double flywheelMotorFollowerCurrent;
    public double flywheelMotorFollowerVoltage;
    public double flywheelMotorFollowerTemperature;
  }

  public default void setFlywheelSpeed(double speed) {
  }

  public default void setFlywheelSpeed(TurretSetpoint setpoint) {
    setFlywheelSpeed(setpoint.flywheelSpeed());
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
