package frc.robot.subsystems.hood;

import edu.wpi.first.math.util.Units;
import frc.robot.util.limits.Limits;
import frc.robot.util.robotswitcher.SwitchableCANID;

public class HoodConstants {
  /** Hood CAN ID. */
  public static final int canId = SwitchableCANID.of(12).get();

  /** Offset from hood encoder zero position to the horizontal. */
  public static final double hoodZeroOffsetRadians = 0;

  /** Angle to which the hood runs when idle, i.e. to fit under the trench. */
  public static final double downAngleRadians = Units.degreesToRadians(0); // TODO

  /** Error tolerance for hood angle closed-loop control. */
  public static final double angleToleranceRadians = Units.degreesToRadians(1); // TODO

  /** Hood angle limits. */
  public static final Limits angleLimitsRadians = new Limits(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
      true);
}
