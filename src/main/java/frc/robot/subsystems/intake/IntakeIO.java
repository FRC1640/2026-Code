package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends AutoCloseable {
  @AutoLog
  public static class IntakeIOInputs {
    public double motorVoltage;
    public double motorTemperature;
    public double motorCurrent;
    public double encoderVelocity;
    public double encoderPosition;

    public double rollerMotorVoltage;
    public double rollerMotorTemperature;
    public double rollerMotorCurrent;
    public double rollerEncoderVelocity;
    public double rollerEncoderPosition;
  }

  public default void updateInputs(IntakeIOInputs inputs) {
  }
  public default void setMotorVoltage(double voltage, IntakeIOInputs inputs) {
  }
  public default void setMotorPosition(double pos, IntakeIOInputs inputs) {
  }
  public default void setRollerMotorVoltage(double voltage, IntakeIOInputs inputs) {
  }
  public default void setRollerVelocity(double velocity, IntakeIOInputs inputs) {
  }

  public default void close() {
  }

}
