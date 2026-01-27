package frc.robot.util.limits;

import edu.wpi.first.math.MathUtil;

/** Class representing a closed interval for mechanism motion. */
public class Limits {
  public final double low;
  public final double high;
  public final int positiveDir;

  /**
   * Constructs a new {@code Limits} using the given range and positive direction.
   *
   * @param limit1
   *            First limit.
   * @param limit2
   *            Second limit.
   * @param positiveDir
   *            Boolean indicating the direction a system will move when a
   *            positive output is applied (for clamping utilities). A value of
   *            {@code true} indicates that positive output moves the system
   *            closer to the high limit, while a {@code false} value means the
   *            system would approach the low limit. (The high limit is simply the
   *            greater of the two.)
   */
  public Limits(double limit1, double limit2, boolean positiveDir) {
    if (limit1 > limit2) {
      this.high = limit1;
      this.low = limit2;
    } else {
      this.low = limit1;
      this.high = limit2;
    }
    this.positiveDir = positiveDir ? 1 : -1;
  }

  public boolean inRange(double value) {
    return value >= low && value <= high;
  }

  public double clampPosition(double position) {
    return MathUtil.clamp(position, low, high);
  }

  public double clampOutput(double position, double output) {
    double outputTowardsHigh = output * positiveDir;
    if (position < low) {
      return Math.min(0, outputTowardsHigh) / positiveDir;
    } else if (position > high) {
      return Math.max(0, outputTowardsHigh) / positiveDir;
    } else {
      return output;
    }
  }
}
