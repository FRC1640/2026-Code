package frc.robot.subsystems.shooter.deflector;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;

public interface DeflectorIO extends AutoCloseable{
  @AutoLog
  public class DeflectorIOInputs{
    public double deflectorAngle;
    public double deflectorMotorTemperature;
    public double deflectorMotorCurrent;
    public double deflectorMotorVoltage;
  }

  public default void setDeflectorAngle(double angle) {
  }

  public default void setDeflectorAngle(TurretSetpoint setpoint) {
    setDeflectorAngle(setpoint.hoodAngle());
  }

  public default void updateInputs(DeflectorIOInputs inputs) {
  }

  @Override
  public default void close() {
  }

  public static DeflectorIO getIOByMode() {
    return switch (Robot.getMode()) {
      case REAL -> new DeflectorIOReal();
      default -> new DeflectorIO() {
      };
    };
  }
}
