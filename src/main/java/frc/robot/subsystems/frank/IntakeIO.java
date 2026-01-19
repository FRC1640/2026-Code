package frc.robot.subsystems.frank;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Robot;

public interface IntakeIO extends AutoCloseable {
    @AutoLog
    public static class IntakeIOInputs {
        public double motorTemperature;
        public double motorCurrent;
        public double motorVoltage;
        public double encoderPosition;
        public double encoderVelocity;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setMotorVoltage(double voltage, IntakeIOInputs inputs) {}

    public default void setMotorPosition(double pos, IntakeIOInputs inputs) {}

    @Override
    public default void close() {}

    public static IntakeIO getIOByMode() {
        return switch (Robot.getMode()) {
            case REAL -> new IntakeIOReal();
            default -> new IntakeIO() {};
        };
    }
}