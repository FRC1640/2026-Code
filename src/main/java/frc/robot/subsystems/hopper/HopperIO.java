package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Robot;

public interface HopperIO extends AutoCloseable{
  @AutoLog
  public class HopperIOInputs{
    public double HopperAngle;
    public double HopperMotorTemperature;
    public double HopperMotorCurrent;
    public double HopperMotorVoltage;
  }

  public default void setHopperVoltage(double voltage) {}

  public default void updateInputs(HopperIOInputs inputs) {}

  @Override
  public default void close() {
  }

  public static HopperIO getIOByMode() {
    return switch (Robot.getMode()) {
      case REAL -> new HopperIOReal();
      case SIM -> new HopperIOSim();
      case REPLAY -> new HopperIO() {};
    };
  }
}
