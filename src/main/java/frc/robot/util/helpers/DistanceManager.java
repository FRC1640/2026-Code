package frc.robot.util.helpers;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DistanceManager {

  /**
   * Returns the nearest distance from the positions from the checked points
   *
   * @param robotPos
   *            The current robot position
   * @param checkPoints
   *            The checking points (A position 2D array with all the positions
   *            that you need to check)
   * @return the distance from the nearest points
   */
  public static double getNearestPositionDistance(Pose2d robotPos, Pose2d[] checkPoints) {

    double distance = Double.MAX_VALUE;
    for (Pose2d pos : checkPoints) {
      double distanceLocalPos = robotPos.getTranslation().getDistance(pos.getTranslation());
      if (distance > distanceLocalPos) {
        distance = distanceLocalPos;
      }
    }
    return distance;
  }

  /**
   * Returns the nearest position from the positions from the checked points
   *
   * @param robotPos
   *            The current robot position
   * @param checkPoints
   *            The checking points (A position 2D array with all the positions
   *            that you need to check)
   * @return the nearest points
   */
  public static Pose2d getNearestPosition(Pose2d robotPos, Pose2d[] checkPoints) {
    double distance = Double.MAX_VALUE;
    Pose2d nearestPos = new Pose2d();
    for (Pose2d pos : checkPoints) {
      double distanceLocalPos = robotPos.getTranslation().getDistance(pos.getTranslation());
      if (distance > distanceLocalPos) {
        distance = distanceLocalPos;
        nearestPos = pos;
      }
    }
    return nearestPos;
  }

  public static Pose2d getNearestPosition(Pose2d robotPos, Pose2d[] checkPoints,
      Function<Pose2d, Pose2d> poseFunction) {
    double distance = Double.MAX_VALUE;
    Pose2d nearestPos = new Pose2d();
    for (Pose2d pos1 : checkPoints) {
      Pose2d pos = poseFunction.apply(pos1);
      double distanceLocalPos = robotPos.getTranslation().getDistance(pos.getTranslation());
      if (distance > distanceLocalPos) {
        distance = distanceLocalPos;
        nearestPos = pos;
      }
    }
    return nearestPos;
  }

  public static double getPositionDistance(Pose2d robotPos, Pose2d targetPos) {
    return robotPos.getTranslation().getDistance(targetPos.getTranslation());
  }

  public static Pose2d addRotatedDim(Pose2d pose2d, double dim, Rotation2d rot) {
    Translation2d translation = pose2d.getTranslation().minus(new Translation2d(dim, 0).rotateBy(rot));
    return new Pose2d(translation, pose2d.getRotation());
  }

  /**
   * There probably is a WPI method for this, I'm too bored to find it Checks if
   * something is in range
   *
   * @param error
   *            The acceptable error between the two value
   * @param value1
   *            Value 1
   * @param value2
   *            Value 2
   */
  public static boolean inRange(double error, double value1, double value2) {
    return error >= Math.abs(value1 - value2);
  }
  /**
   * others if a pose is ABOVE other pose
   *
   * @param pose
   *            The pose that you want to other the other is from
   * @param other
   *            The pose that you want to see is above the pose
   */
  public static boolean isAboveOf(Pose2d pose, Pose2d other) {
    return pose.getY() > other.getY();
  }
  /**
   * others if a pose is ABOVE other pose
   *
   * @param pose
   *            The pose that you want to other the other is from
   * @param other
   *            The pose that you want to see is below the pose
   */
  public static boolean isBelowOf(Pose2d pose, Pose2d other) {
    return pose.getY() < other.getY();
  }
  /**
   * others if a pose is ABOVE other pose
   *
   * @param pose
   *            The pose that you want to other the other is from
   * @param other
   *            The pose that you want to see is right of the pose
   */
  public static boolean isRightOf(Pose2d pose, Pose2d other) {
    return pose.getX() > other.getX();
  }
  /**
   * others if a pose is LEFT the other
   *
   * @param pose
   *            The pose that you want to other the other is from
   * @param other
   *            The pose that you want to see is left of the pose
   */
  public static boolean isLeftOf(Pose2d pose, Pose2d other) {
    return pose.getX() < other.getX();
  }
}

