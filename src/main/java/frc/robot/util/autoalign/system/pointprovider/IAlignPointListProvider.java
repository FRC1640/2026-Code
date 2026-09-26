package frc.robot.util.autoalign.system.pointprovider;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public interface IAlignPointListProvider extends IAlignPointProvider {
  List<Pose2d> getGoalPoses();
}
