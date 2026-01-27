package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Robot;

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

  public default void setTurretVoltage(double voltage) {
  }

  public default void updateInputs(TurretIOInputs inputs) {
  }

  @Override
  public default void close() {
  }

  public static TurretIO getIOByMode() {
    return switch (Robot.getMode()) {
      case REAL -> new TurretIOReal();
      case SIM -> new TurretIOSim();
      case REPLAY -> new TurretIO() {
      };
    };
  }
}
