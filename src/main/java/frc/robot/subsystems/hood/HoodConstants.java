package frc.robot.subsystems.hood;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotConstants.RobotTypes;
import frc.robot.util.limits.Limits;
import frc.robot.util.robotswitcher.Switchable;
import frc.robot.util.robotswitcher.SwitchableCANID;

public class HoodConstants {
  /** Hood CAN ID. */
  public static final int canId = SwitchableCANID.of(12).get();

  /** Hood angle limits, with the horizontal. */
  public static final Limits angleLimitsRadians = Switchable
      .of(new Limits(Units.degreesToRadians(27), Units.degreesToRadians(50), true))
      .addAlt(RobotTypes.duex26, new Limits(Units.degreesToRadians(15), Units.degreesToRadians(29), true)).get();

  /**
   * Additional offset added to encoder after zeroing, as a buffer against
   * discontinuities.
   */
  public static final double hoodEncoderManualOffset = 0.1;

  // Don't change unless the encoder-to-angle conversion needs to be remeasured!
  // These are
  // NOT the same as lower and upper limits!
  /**
   * Lower encoder count used in calculating an encoder-to-angle conversion ratio.
   * This is the encoder count for which the hood has angle
   * {@link HoodConstants#hoodAngle0Radians}.
   */
  public static final double hoodEncoderCount0 = 0 + hoodEncoderManualOffset;
  /**
   * Upper encoder count used in calculating an encoder-to-angle conversion ratio.
   * This is the encoder count for which the hood has angle
   * {@link HoodConstants#hoodAngle1Radians}.
   */
  public static final double hoodEncoderCount1 = Switchable.of(0.409).addAlt(RobotTypes.duex26, 0.745).get()
      + hoodEncoderManualOffset;
  /**
   * Lower angle used in calculating an encoder-to-angle conversion ratio. This is
   * the angle at which the hood encoder reads
   * {@link HoodConstants#hoodEncoderCount0}
   */
  public static final double hoodAngle0Radians = Switchable.of(Units.degreesToRadians(26))
      .addAlt(RobotTypes.duex26, Units.degreesToRadians(14)).get();
  /**
   * Upper angle used in calculating an encoder-to-angle conversion ratio. This is
   * the angle at which the hood encoder reads
   * {@link HoodConstants#hoodEncoderCount1}.
   */
  public static final double hoodAngle1Radians = Switchable.of(Units.degreesToRadians(40))
      .addAlt(RobotTypes.duex26, Units.degreesToRadians(35)).get();

  /** Offset from the horizontal to hood encoder zero position. */
  public static final double hoodZeroOffsetRadians = Switchable.of(Units.degreesToRadians(26))
      .addAlt(RobotTypes.duex26, Units.degreesToRadians(14)).get();

  // custom formatting
  /** Conversion factor from encoder counts to radian angle. */
  public static final double hoodEncoderToAngleRatio =
      (hoodAngle1Radians - hoodAngle0Radians)
      / (hoodEncoderCount1 - hoodEncoderCount0);
  // spotless formatting

  /** Angle to which the hood runs when idle, i.e. to fit under the trench. */
  public static final double downAngleRadians = Switchable.of(Units.degreesToRadians(27))
      .addAlt(RobotTypes.duex26, Units.degreesToRadians(15)).get(); // TODO

  /** Error tolerance for hood angle closed-loop control. */
  public static final double angleToleranceRadians = Units.degreesToRadians(1); // TODO
}
