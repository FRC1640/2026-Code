package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends AutoCloseable {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeMotorVoltage;
    public double intakeMotorTemperature;
    public double intakeMotorCurrent;
    public double intakeEncoderVelocity;
    public double intakeEncoderPosition;
  }

  public default void updateInputs(IntakeIOInputs inputs) {
  }
  public default void setVoltage(double voltage, IntakeIOInputs inputs) {
  }
  public default void setPosition(double pos, IntakeIOInputs inputs) {
  }

  public default void close() {
  }

}
