package frc.robot.subsystems.frank;

public class IntakeConstants {
    public static final int canID = 13;
    public static final double gearRatio = 1;

    // zero position: 35 deg inward from vertical
    public static final double intakeHandoffPosition = 0.982; // 0.982
    public static final double intakeGroundPosition = 0.59; // 0.563
    public static final double troughPosition = 0.850;
    public static final double safePosition = 0.881;

    public static final double intakeUpperLimit = 0.98;
    public static final double intakeLowerLimit = 0.56;

    public static final double relativeToAbsoluteConversionRatio = 0.0121278;
    public static final double intakeRelativeSetpointThreshold = 0.3;
}