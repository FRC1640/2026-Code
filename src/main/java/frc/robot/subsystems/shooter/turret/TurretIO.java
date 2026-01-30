package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Robot;

public interface TurretIO extends AutoCloseable {
  @AutoLog
  public class TurretIOInputs {
    public double angle;
    public double angularVelocity;
    public double motorCurrent;
    public double motorVoltage;
    public double motorTemperature;
  }

  public default void setTurretState(double angle, double angularVelocity) {
  }

  public default void setVoltage(double voltage) {
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
