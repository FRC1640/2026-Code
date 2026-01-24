package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO extends AutoCloseable {
    @AutoLog
    public static class IndexerIOInputs {
        public double indexerMotorVelocity = 0.0;
        public double indexerMotorVoltage = 0.0;
        public double indexerMotorCurrent = 0.0;
        public double indexerMotorTemperature = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}
    
    public default void setIndexerMotorVoltage(double voltage) {}

    public default void resetEncoder() {}

    @Override
    default void close() {}
}
