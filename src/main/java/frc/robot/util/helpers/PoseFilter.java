package frc.robot.util.helpers;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.tuple.Tuple2;

public class PoseFilter {

  public enum PoseFilterType {
    ABOVE, BELOW, LEFT, RIGHT
  }

  public List<Tuple2<PoseFilterType, Pose2d>> checks = new ArrayList<>();
  public List<Tuple2<Translation2d, Pose2d>> transChecks = new ArrayList<>();

  public PoseFilter(PoseFilterType filter, Pose2d checkpose) {
    checks.add(new Tuple2<>(filter, checkpose));
  }

  public PoseFilter(Translation2d normal, Pose2d checkpose) {
    transChecks.add(new Tuple2<>(normal, checkpose));
  }

  public PoseFilter addFilter(PoseFilterType filter, Pose2d checkpose) {
    checks.add(new Tuple2<>(filter, checkpose));
    return this;
  }

  public PoseFilter addFilter(Translation2d normal, Pose2d checkpose) {
    transChecks.add(new Tuple2<>(normal, checkpose));
    return this;
  }
  // TODO add angle

  public boolean poseSatisfies(Pose2d pose) {
    boolean satis = true;
    for (var check : transChecks) {
      satis = (((pose.getTranslation().minus(check.b.getTranslation())).dot(check.a)) > 0);
    }
    loop : for (var c : checks) {
      switch (c.a) {
        case ABOVE -> {
          if (!DistanceManager.isAboveOf(pose, c.b)) {
            satis = false;
            break loop;
          }
        }
        case BELOW -> {
          if (!DistanceManager.isBelowOf(pose, c.b)) {
            satis = false;
            break loop;
          }
        }
        case LEFT -> {
          if (!DistanceManager.isLeftOf(pose, c.b)) {
            satis = false;
            break loop;
          }
        }s
        case RIGHT -> {
          if (!DistanceManager.isRightOf(pose, c.b)) {
            satis = false;
            break loop;
          }
        }
        default -> {
        }
      }
    }
    return satis;
  }
}
