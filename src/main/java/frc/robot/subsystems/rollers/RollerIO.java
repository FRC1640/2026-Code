package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO extends AutoCloseable {
  @AutoLog
  public static class RollerIOInputs {

    public double rollerMotorVoltage;
    public double rollerMotorTemperature;
    public double rollerMotorCurrent;
    public double rollerEncoderVelocity;
  }

  public default void updateInputs(RollerIOInputs inputs) {
  }
  public default void setVoltage(double voltage, RollerIOInputs inputs) {
  }
  public default void setVelocity(double velocity, RollerIOInputs inputs) {
  }

  public default void close() {
  }

}
