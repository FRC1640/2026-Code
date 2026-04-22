package frc.robot.util.helpers;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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

  /**
   * Will Pass Point Shoots two vectors to predicted pose and current pose of the
   * robot, and takes dot product. Returns true if dot product is >0
   *
   * @param checkPose
   *            The pose to check for
   * @param robotPose
   *            The robot's pose
   * @param chassisSpeeds
   *            The robot's chassis speeds
   * @param lookahead
   *            The time that it looks ahead for
   */
  public static boolean willPassPoint(Pose2d checkPose, Translation2d normal, Pose2d robotPose,
      ChassisSpeeds chassisSpeeds, double lookahead) {
    Pose2d predicted = robotPose.exp(chassisSpeeds.toTwist2d(lookahead));
    Translation2d robotPoseTransform = robotPose.getTranslation().minus(checkPose.getTranslation());
    Translation2d predictedTransform = predicted.getTranslation().minus(checkPose.getTranslation());
    return robotPoseTransform.dot(normal) * predictedTransform.dot(normal) < 0;
  }

  public static double angleDistance(double angle1, double angle2) {
    double rawDifference = angle2 - angle1;
    // Normalize the difference within the range [-pi, pi]
    double normalizedDifference = ((rawDifference + Math.PI) % (2 * Math.PI)) - Math.PI;

    // Adjust for cases where the difference is exactly -pi
    if (normalizedDifference < -Math.PI) {
      normalizedDifference += 2 * Math.PI;
    }
    return normalizedDifference;
  }

  // custom format
  /**
   * Calculates the distance by which a polygon extends in some direction from the coordinate system origin.
   *
   * @param vertices Vertices of the polygon, with respect to an origin.
   * @param direction Direction in which to calculate protrusion.
   * @return Length of protrusion.
   * 
   */ // spotless format
  public static double calculatePolygonProtrusion(Translation2d[] vertices, Translation2d direction) {
    return calculatePolygonProtrusion(vertices, Translation2d.kZero, direction);
  }

  // custom format
  /**
   * Calculates the distance by which a polygon extends in some direction from a reference point.
   *
   * @param vertices Vertices of the polygon, with respect to an origin.
   * @param point Point against which to check distance, with respect to the same origin.
   * @param direction Direction in which to calculate protrusion.
   * @return Length of protrusion.
   * 
   */ // spotless format
  public static double calculatePolygonProtrusion(Translation2d[] vertices, Translation2d point,
      Translation2d direction) {
    Translation2d normalizedDirection = direction.div(direction.getNorm());
    double maxProtrusion = 0;
    for (Translation2d vertex : vertices) {
      Translation2d vertexDelta = vertex.minus(point);
      double vertexProjection = vertexDelta.dot(normalizedDirection);
      if (vertexProjection > maxProtrusion) {
        maxProtrusion = vertexProjection;
      }
    }
    return maxProtrusion;
  }

}
