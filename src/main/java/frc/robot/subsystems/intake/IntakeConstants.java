package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.util.limits.Limits;
import frc.robot.util.robotswitcher.SwitchableCANID;

public class IntakeConstants {
  /** Intake CAN ID. */
  public static final int canId = SwitchableCANID.of(15).get();

  /** Offset from intake encoder zero position to the vertical. */
  public static final double intakeZeroOffsetRadians = Units.degreesToRadians(151); // TODO

  /** Intaking (down) position with the vertical. */
  public static final double activePositionRadians = Units.degreesToRadians(15); // TODO
  /** Stowed (up) position with the vertical. */
  public static final double stowedPositionRadians = Units.degreesToRadians(150); // TODO

  public static final double intakeManualOffset = 0.1;

  public static final double intakeMaxAngleRadians = Units.degreesToRadians(136);
  public static final double intakeMinAngleRadians = Units.degreesToRadians(15);
  public static final double intakeMaxEncoderCount = intakeManualOffset + 0.335;
  public static final double intakeMinEncoderCount = 0 + intakeManualOffset;

  public static final double intakeEncoderToRadiansConversion = (intakeMaxAngleRadians - intakeMinAngleRadians)
      / (intakeMaxEncoderCount - intakeMinEncoderCount);

  /** Intake position limits. */
  public static final Limits positionLimitsRadians = new Limits(Units.degreesToRadians(15),
      Units.degreesToRadians(136), true);
}
