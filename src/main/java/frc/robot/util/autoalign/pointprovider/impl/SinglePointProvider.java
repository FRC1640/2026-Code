package frc.robot.util.autoalign.pointprovider.impl;

import java.util.function.Supplier;


import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.autoalign.pointprovider.IAlignPointProvider;

public class SinglePointProvider implements IAlignPointProvider {
  Supplier<Pose2d> singlePointTarget, robotPose;
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
  public void onComplete() {
  }
}
