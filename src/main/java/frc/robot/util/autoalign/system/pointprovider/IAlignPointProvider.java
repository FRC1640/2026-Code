package frc.robot.util.autoalign.system.pointprovider;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.autoalign.system.controller.IAlignController;

public interface IAlignPointProvider {
  Pose2d getTargetPose();
  Pose2d getRobotPose();
  boolean hasPoint();
  void attachAlignController(IAlignController alignController);
}
