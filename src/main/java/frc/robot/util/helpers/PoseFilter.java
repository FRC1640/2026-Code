package frc.robot.util.helpers;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.tuple.Tuple2;

public class PoseFilter {

  public enum PoseFilterType {
    ABOVE, BELOW, LEFT, RIGHT
  }

  public List<Tuple2<PoseFilterType, Pose2d>> checks;

  public PoseFilter(PoseFilterType filter, Pose2d checkpose) {
    checks.add(new Tuple2<>(filter, checkpose));
  }
  public PoseFilter addFilter(PoseFilterType filter, Pose2d checkpose) {
    checks.add(new Tuple2<>(filter, checkpose));
    return this;
  }
  // TODO add angle
  public boolean poseSatisfies(Pose2d pose) {
    boolean satis = true;
    loop : for (var c : checks) {
      switch (c.valA) {
        case ABOVE -> {
          if (!DistanceManager.isAboveOf(c.valB, pose)) {
            satis = false;
            break loop;
          }
        }
        case BELOW -> {
          if (!DistanceManager.isBelowOf(c.valB, pose)) {
            satis = false;
            break loop;
          }
        }
        case LEFT -> {
          if (!DistanceManager.isLeftOf(c.valB, pose)) {
            satis = false;
            break loop;
          }
        }
        case RIGHT -> {
          if (!DistanceManager.isRightOf(c.valB, pose)) {
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
