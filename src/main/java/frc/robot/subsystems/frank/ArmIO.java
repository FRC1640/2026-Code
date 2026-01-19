package frc.robot.subsystems.frank;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Robot;

public interface ArmIO extends AutoCloseable {
  @AutoLog
  public class ArmIOInputs {
    public double position;
    public double velocity;
    public double current;
    public double temperature;
  }

  public default void setMotorVoltage(double voltage) {}
  public default void setMotorPosition(double pos) {}

  public default void updateInputs(ArmIOInputs inputs) {}

  @Override
  public default void close() {}

  public static ArmIO getIOByMode() {
    return switch (Robot.getMode()) {
      case REAL -> new ArmIOReal();
      default -> new ArmIO() {};
    };
  }
}
