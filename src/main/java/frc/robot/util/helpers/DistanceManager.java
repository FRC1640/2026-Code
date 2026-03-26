package frc.robot.util.helpers;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DistanceManager {
  /**
   * Returns the distance from the robot to the closest checkpoint.
   *
   * @param robotPos
   *            The robot's current pose
   * @param checkPoints
   *            Array of poses to evaluate
   * @return The minimum distance to any checkpoint
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
   * Returns the checkpoint pose closest to the robot.
   *
   * @param robotPos
   *            The robot's current pose
   * @param checkPoints
   *            Array of poses to evaluate
   * @return The nearest pose from the array
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

  /**
   * Returns the closest transformed pose from a set of checkpoints.
   *
   * @param robotPos
   *            The robot's current pose
   * @param checkPoints
   *            Array of poses to evaluate
   * @param poseFunction
   *            Function applied to each pose before distance calculation
   * @return The nearest transformed pose
   */
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

  /**
   * Computes the distance between the robot and a target pose.
   *
   * @param robotPos
   *            The robot's current pose
   * @param targetPos
   *            The target pose
   * @return Distance between the two poses
   */
  public static double getPositionDistance(Pose2d robotPos, Pose2d targetPos) {
    return robotPos.getTranslation().getDistance(targetPos.getTranslation());
  }

  /**
   * Offsets a pose by a given distance along a rotated axis.
   *
   * @param pose2d
   *            The original pose
   * @param dim
   *            Distance to offset
   * @param rot
   *            Rotation defining the offset direction
   * @return The transformed pose
   */
  public static Pose2d addRotatedDim(Pose2d pose2d, double dim, Rotation2d rot) {
    Translation2d translation = pose2d.getTranslation().minus(new Translation2d(dim, 0).rotateBy(rot));
    return new Pose2d(translation, pose2d.getRotation());
  }

  /**
   * Checks if one pose is above another (greater Y value).
   *
   * @param pose
   *            The reference pose
   * @param other
   *            The pose to compare
   * @return True if pose is above other
   */
  public static boolean isAboveOf(Pose2d pose, Pose2d other) {
    return pose.getY() > other.getY();
  }

  /**
   * Checks if one pose is below another (smaller Y value).
   *
   * @param pose
   *            The reference pose
   * @param other
   *            The pose to compare
   * @return True if pose is below other
   */
  public static boolean isBelowOf(Pose2d pose, Pose2d other) {
    return pose.getY() < other.getY();
  }

  /**
   * Checks if one pose is to the right of another (greater X value).
   *
   * @param pose
   *            The reference pose
   * @param other
   *            The pose to compare
   * @return True if pose is to the right of other
   */
  public static boolean isRightOf(Pose2d pose, Pose2d other) {
    return pose.getX() > other.getX();
  }

  /**
   * Checks if one pose is to the left of another (smaller X value).
   *
   * @param pose
   *            The reference pose
   * @param other
   *            The pose to compare
   * @return True if pose is to the left of other
   */
  public static boolean isLeftOf(Pose2d pose, Pose2d other) {
    return pose.getX() < other.getX();
  }

  /**
   * Determines whether the robot will pass a given point within a lookahead time.
   *
   * This uses the dot product of the current and predicted position vectors
   * relative to a reference normal. A sign change indicates the robot crosses the
   * plane defined by the point and normal.
   *
   * @param checkPose
   *            The reference point to check against
   * @param normal
   *            The normal vector defining the boundary
   * @param robotPose
   *            The robot's current pose
   * @param chassisSpeeds
   *            The robot's current chassis speeds
   * @param lookahead
   *            Time horizon for prediction
   * @return True if the robot will pass the point
   */
  public static boolean willPassPoint(Pose2d checkPose, Translation2d normal, Pose2d robotPose,
      ChassisSpeeds chassisSpeeds, double lookahead) {

    Pose2d predicted = robotPose.exp(chassisSpeeds.toTwist2d(lookahead));
    Translation2d robotPoseTransform = robotPose.getTranslation().minus(checkPose.getTranslation());
    Translation2d predictedTransform = predicted.getTranslation().minus(checkPose.getTranslation());

    return robotPoseTransform.dot(normal) * predictedTransform.dot(normal) < 0;
  }

  public static boolean isBetweenPose(Pose2d point1, Pose2d point2, Pose2d check) {
    return Math.min(point1.getX(), point2.getX()) < check.getX()
        && Math.max(point1.getX(), point2.getX()) > check.getX()
        && Math.min(point1.getY(), point2.getY()) < check.getY()
        && Math.max(point1.getY(), point2.getY()) > check.getY();
  }

}
