package frc.robot.util.helpers;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PoseFilter {
  List<Function<Pose2d, BooleanSupplier>> filters = new ArrayList<>();
  List<PoseFilter> orFilter = new ArrayList<>();

  public PoseFilter(Function<Pose2d, BooleanSupplier> initialCondition) {
    filters.add(initialCondition);
  }

  public boolean poseSatisfies(Pose2d pose) {
    return filters.stream().allMatch((x) -> x.apply(pose).getAsBoolean())
        || orFilter.stream().anyMatch((x) -> x.poseSatisfies(pose));
  }

  public PoseFilter addFilter(Function<Pose2d, BooleanSupplier> condition) {
    filters.add(condition);
    return this;
  }

  public PoseFilter addFilter(Translation2d trans, Pose2d pose) {
    addFilter(PoseFilterPreset.vectorFilterFunction(trans, pose));
    return this;
  }

  public PoseFilter or(PoseFilter poseFilter) {
    orFilter.add(poseFilter);
    return this;
  }

  public class PoseFilterPreset {
    public static Function<Pose2d, BooleanSupplier> isPoseAboveOf(Supplier<Pose2d> pose) {
      return new Function<Pose2d, BooleanSupplier>() {
        @Override
        public BooleanSupplier apply(Pose2d t) {
          return () -> DistanceManager.isAboveOf(t, pose.get());
        }
      };
    }

    public static Function<Pose2d, BooleanSupplier> isPoseBelowOf(Supplier<Pose2d> pose) {
      return new Function<Pose2d, BooleanSupplier>() {
        @Override
        public BooleanSupplier apply(Pose2d t) {
          return () -> DistanceManager.isBelowOf(t, pose.get());
        }
      };
    }

    public static Function<Pose2d, BooleanSupplier> isPoseRightOf(Supplier<Pose2d> pose) {
      return new Function<Pose2d, BooleanSupplier>() {
        @Override
        public BooleanSupplier apply(Pose2d t) {
          return () -> DistanceManager.isRightOf(t, pose.get());
        }
      };
    }

    public static Function<Pose2d, BooleanSupplier> isPoseLeftOf(Supplier<Pose2d> pose) {
      return new Function<Pose2d, BooleanSupplier>() {
        @Override
        public BooleanSupplier apply(Pose2d t) {
          return () -> DistanceManager.isLeftOf(t, pose.get());
        }
      };
    }

    public static Function<Pose2d, BooleanSupplier> vectorFilterFunction(Supplier<Translation2d> trans,
        Supplier<Pose2d> pose) {
      return new Function<Pose2d, BooleanSupplier>() {
        @Override
        public BooleanSupplier apply(Pose2d t) {
          return () -> (((t.getTranslation().minus(pose.get().getTranslation())).dot(trans.get())) > 0);
        }
      };
    }

    public static Function<Pose2d, BooleanSupplier> inRectZone(Supplier<Pose2d> point1, Supplier<Pose2d> point2) {
      return new Function<Pose2d, BooleanSupplier>() {
        @Override
        public BooleanSupplier apply(Pose2d t) {
          return () -> DistanceManager.isBetweenPose(point1.get(), point2.get(), t);
        }
      };
    }

    public static Function<Pose2d, BooleanSupplier> isPoseAboveOf(Pose2d pose) {
      return new Function<Pose2d, BooleanSupplier>() {
        @Override
        public BooleanSupplier apply(Pose2d t) {
          return () -> DistanceManager.isAboveOf(t, pose);
        }
      };
    }

    public static Function<Pose2d, BooleanSupplier> isPoseBelowOf(Pose2d pose) {
      return new Function<Pose2d, BooleanSupplier>() {
        @Override
        public BooleanSupplier apply(Pose2d t) {
          return () -> DistanceManager.isBelowOf(t, pose);
        }
      };
    }

    public static Function<Pose2d, BooleanSupplier> isPoseRightOf(Pose2d pose) {
      return new Function<Pose2d, BooleanSupplier>() {
        @Override
        public BooleanSupplier apply(Pose2d t) {
          return () -> DistanceManager.isRightOf(t, pose);
        }
      };
    }

    public static Function<Pose2d, BooleanSupplier> isPoseLeftOf(Pose2d pose) {
      return new Function<Pose2d, BooleanSupplier>() {
        @Override
        public BooleanSupplier apply(Pose2d t) {
          return () -> DistanceManager.isLeftOf(t, pose);
        }
      };
    }

    public static Function<Pose2d, BooleanSupplier> vectorFilterFunction(Translation2d trans, Pose2d pose) {
      return new Function<Pose2d, BooleanSupplier>() {
        @Override
        public BooleanSupplier apply(Pose2d t) {
          return () -> (((t.getTranslation().minus(pose.getTranslation())).dot(trans)) > 0);
        }
      };
    }

    public static Function<Pose2d, BooleanSupplier> inRectZone(Pose2d point1, Pose2d point2) {
      return new Function<Pose2d, BooleanSupplier>() {
        @Override
        public BooleanSupplier apply(Pose2d t) {
          return () -> DistanceManager.isBetweenPose(point1, point2, t);
        }
      };
    }

  }

}
