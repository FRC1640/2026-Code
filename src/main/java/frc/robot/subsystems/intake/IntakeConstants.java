package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.util.limits.Limits;
import frc.robot.util.robotswitcher.Switchable;
import frc.robot.util.robotswitcher.SwitchableCANID;

public class IntakeConstants {
  /** Intake CAN ID. */
  public static final int canId = SwitchableCANID.of(15).get();

  /** Offset from the horizontal to intake encoder zero position. */
  public static final double intakeZeroOffsetRadians = Units.degreesToRadians(15);

  /** Intaking (down) position with the horizontal. */
  public static final double activePositionRadians = Units.degreesToRadians(15);
  /** Stowed (up) position with the horizontal. */
  public static final double stowedPositionRadians = Units.degreesToRadians(136); // TODO

  /**
   * Additional offset added to encoder after zeroing, as a buffer against
   * discontinuities.
   */
  public static final double intakeManualOffset = 0.1; // TODO make negative?

  public static final double intakeAngle1Radians = Units.degreesToRadians(136);
  public static final double intakeAngle0Radians = Units.degreesToRadians(15);
  public static final double intakeEncoderCount1 = Switchable.of(0.335 + intakeManualOffset).get();
  public static final double intakeEncoderCount0 = Switchable.of(0 + intakeManualOffset).get();

  public static final double intakeEncoderToRadiansConversion = (intakeAngle1Radians - intakeAngle0Radians)
      / (intakeEncoderCount1 - intakeEncoderCount0);

  public static final double intakeSetpointToleranceRadians = Units.degreesToRadians(4);

  /** Intake position limits. */
  public static final Limits positionLimitsRadians = new Limits(Units.degreesToRadians(15),
      Units.degreesToRadians(136), true);
}
