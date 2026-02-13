package frc.robot.subsystems.intakeRollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO extends AutoCloseable {
  @AutoLog
  public static class IntakeRollerIOInputs {

    public double motorVoltage;
    public double motorTemperature;
    public double motorCurrent;
    public double encoderVelocity;
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {
  }

  public default void runVoltage(double voltage) {
  }

  public default void runVelocity(double velocity) {
  }

  @Override
  public default void close() {
  }

}
