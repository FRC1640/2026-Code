package frc.robot.util.autoalign.pointprovider;

import edu.wpi.first.math.geometry.Pose2d;

public interface IAlignPointProvider {
  Pose2d getTargetPose();
  Pose2d getRobotPose();
  boolean hasPoint();
  void onComplete();
}
