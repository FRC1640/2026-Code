package frc.robot.util.autoalign.system.pointprovider.impl;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.autoalign.system.controller.AlignControllerBase;
import frc.robot.util.autoalign.system.pointprovider.IAlignPointProvider;
import frc.robot.util.helpers.DistanceManager;

public class NearestPointProvider implements IAlignPointProvider {
    Supplier<List<Pose2d>> points;
    Supplier<Pose2d> robotPose;
    AlignControllerBase alignController;

    public NearestPointProvider(Supplier<List<Pose2d>> pointSupplier, Supplier<Pose2d> robotPose) {
        this.points = pointSupplier;
        this.robotPose = robotPose;
    }

    @Override
    public Pose2d getTargetPose() {
        return DistanceManager.getNearestPosition(getRobotPose(), points.get());
    }

    @Override
    public Pose2d getRobotPose() {
       return robotPose.get();
    }

    @Override
    public boolean hasPoint() {
        return getTargetPose() != null;
    }

    @Override
    public void attachAlignController(AlignControllerBase alignController) {
        this.alignController = alignController;
    }
    
}
