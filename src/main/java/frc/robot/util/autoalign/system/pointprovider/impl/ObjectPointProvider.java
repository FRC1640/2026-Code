package frc.robot.util.autoalign.system.pointprovider.impl;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.sensors.objectdetection.PhotonObjectDetected;
import frc.robot.util.autoalign.system.controller.AlignControllerBase;
import frc.robot.util.autoalign.system.pointprovider.IAlignPointProvider;
import frc.robot.util.helpers.DistanceManager;

public class ObjectPointProvider implements IAlignPointProvider {
    Supplier<Pose2d> robotPose;
    Supplier<PhotonObjectDetected[]> photonObjects;
    public ObjectPointProvider(Supplier<Pose2d> robotPose, Supplier<PhotonObjectDetected[]> photonObjects) {
        this.robotPose = robotPose;
        this.photonObjects = photonObjects;
    }

    @Override
    public Pose2d getTargetPose() {
        return DistanceManager.getNearestPosition(getRobotPose(), Stream.of(photonObjects.get()).map((x) -> x.pose().toPose2d()).toList());
    }

    @Override
    public Pose2d getRobotPose() {
        return robotPose.get();
    }

    @Override
    public boolean hasPoint() {
        return photonObjects.get().length > 0;
    }

    @Override
    public void attachAlignController(AlignControllerBase alignController) {
    }
    
}
