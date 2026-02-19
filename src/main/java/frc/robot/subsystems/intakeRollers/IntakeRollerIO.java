package frc.robot.subsystems.intakeRollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO extends AutoCloseable {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public double motorVoltage;
    public double motorTemperatureCelsius;
    public double motorCurrent;
    public double encoderVelocityRadiansPerSecond;
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {
  }

  public default void setVoltage(double voltage) {
  }

  public default void setVelocity(double velocity) {
  }

  @Override
  public default void close() {
  }

}
