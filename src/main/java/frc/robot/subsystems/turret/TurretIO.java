package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Robot;

public interface TurretIO extends AutoCloseable {
  @AutoLog
  public class TurretIOInputs {
    public double turretAngle;
    public double turretAngularVelocity;
    public double turretMotorTemperature;
    public double turretMotorCurrent;
  }

  public default void setTurretState(TurretSetpoint setpoint) {}

  public default void updateInputs(TurretIOInputs inputs) {}

  @Override
  public default void close() {}

  public static record TurretSetpoint(double turretAngle, double turretOmega, double hoodAngle, double flywheelSpeed) {}

  public static TurretIO getIOByMode() {
    return switch (Robot.getMode()) {
      case REAL -> new TurretIOReal();
      default -> new TurretIO() {};
    };
  }
}
