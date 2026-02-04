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

    public double rollerMotorVoltage;
    public double rollerMotorTemperature;
    public double rollerMotorCurrent;
    public double rollerEncoderVelocity;
  }

  public default void updateInputs(IntakeIOInputs inputs) {
  }
  public default void setIntakeVoltage(double voltage, IntakeIOInputs inputs) {
  }
  public default void setIntakePosition(double pos, IntakeIOInputs inputs) {
  }
  public default void setRollerVoltage(double voltage, IntakeIOInputs inputs) {
  }
  public default void setRollerVelocity(double velocity, IntakeIOInputs inputs) {
  }
  public default void runVoltages(double intakeVoltage, double rollerVoltage, IntakeIOInputs inputs) {
  }

  public default void close() {
  }

}
