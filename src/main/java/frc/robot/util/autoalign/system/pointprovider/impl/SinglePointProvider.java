package frc.robot.util.autoalign.system.pointprovider.impl;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.autoalign.system.controller.AlignControllerBase;
import frc.robot.util.autoalign.system.eventshandler.events.ECompleteReachPoint;
import frc.robot.util.autoalign.system.pointprovider.IAlignPointProvider;

public class SinglePointProvider implements IAlignPointProvider {
  Supplier<Pose2d> singlePointTarget, robotPose;
  AlignControllerBase alignController;
  public SinglePointProvider(Supplier<Pose2d> singlePointTarget, Supplier<Pose2d> robotPose) {
    this.singlePointTarget = singlePointTarget;
    this.robotPose = robotPose;
  }
  @Override
  public Pose2d getTargetPose() {
    return singlePointTarget.get();
  }

  @Override
  public Pose2d getRobotPose() {
    return robotPose.get();
  }

  @Override
  public boolean hasPoint() {
    return getRobotPose() != null;
  }

  @Override
  public void attachAlignController(AlignControllerBase alignController) {
    this.alignController = alignController;
    alignController.getAlignSystemEventBus().registerListener(ECompleteReachPoint.class, this::onComplete);
  }
  void onComplete(ECompleteReachPoint complete) {
  }
}
