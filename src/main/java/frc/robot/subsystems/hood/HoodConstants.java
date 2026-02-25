package frc.robot.subsystems.hood;

import edu.wpi.first.math.util.Units;
import frc.robot.util.limits.Limits;
import frc.robot.util.robotswitcher.SwitchableCANID;

public class HoodConstants {
  /** Hood CAN ID. */
  public static final int canId = SwitchableCANID.of(12).get();

  /** Hood angle limits, with the horizontal. */
  public static final Limits angleLimitsRadians = new Limits(Units.degreesToRadians(29), Units.degreesToRadians(90),
      true);

  public static final double hoodEncoderManualOffset = 0.1;

  public static final double hoodMaxEncoderCount = 0.745 + hoodEncoderManualOffset;
  public static final double hoodMinEncoderCount = 0 + hoodEncoderManualOffset;
  public static final double hoodMaxAngleRadians = Units.degreesToRadians(45);
  public static final double hoodMinAngleRadians = Units.degreesToRadians(24);

  /** Offset from hood encoder zero position to the horizontal. */
  public static final double hoodZeroOffsetRadians = Units.degreesToRadians(25);

  // custom formatting
  /** Conversion factor from encoder counts to radian angle. */
  public static final double hoodEncoderToAngleRatio =
      (hoodMaxAngleRadians - hoodMinAngleRadians)
      / (hoodMaxEncoderCount - hoodMinEncoderCount);
  // spotless formatting

  /** Angle to which the hood runs when idle, i.e. to fit under the trench. */
  public static final double downAngleRadians = Units.degreesToRadians(16); // TODO

  /** Error tolerance for hood angle closed-loop control. */
  public static final double angleToleranceRadians = Units.degreesToRadians(1); // TODO
}
