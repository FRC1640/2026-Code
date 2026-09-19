package frc.robot.util.autoalign.pointprovider;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public interface IAlignPointListProvider extends IAlignPointProvider {
  List<Pose2d> getGoalPoses();
}
