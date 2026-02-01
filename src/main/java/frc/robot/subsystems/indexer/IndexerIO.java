package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Robot;

public interface IndexerIO extends AutoCloseable {
  @AutoLog
  public static class IndexerIOInputs {
    public double motorVelocity = 0.0;
    public double motorVoltage = 0.0;
    public double motorCurrent = 0.0;
    public double motorTemperature = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {
  }

  public default void setIndexerMotorVoltage(double voltage) {
  }

  @Override
  default void close() {
  }

  public static IndexerIO getIOByMode() {
    return switch (Robot.getMode()) {
      case REAL -> new IndexerIOReal();
      case SIM -> new IndexerIOSim();
      case REPLAY -> new IndexerIO() {
      };
    };
  }
}
