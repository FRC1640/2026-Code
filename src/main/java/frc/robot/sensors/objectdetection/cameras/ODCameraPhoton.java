package frc.robot.sensors.objectdetection.cameras;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.FieldConstants;
import frc.robot.sensors.objectdetection.ObjectDetectionUtils;
import frc.robot.sensors.objectdetection.PhotonObjectDetected;

public interface ODCameraPhoton {
  List<PhotonPipelineResult> getResults();

  ObjectDetectionCameraConstant getCameraConfig();

  default List<PhotonObjectDetected> update(Pose2d robotPose) {
    List<PhotonPipelineResult> photonResults = getResults();
    List<PhotonTrackedTarget> targets = new ArrayList<>();
    List<Pose3d> objectPoses = new ArrayList<>();
    List<PhotonObjectDetected> photonObjects = new ArrayList<>();

    for (PhotonPipelineResult result : photonResults) {
      for (int i = 0; i < result.getTargets().size(); i++) {
        PhotonTrackedTarget target = result.getTargets().get(i);
        targets.add(target);
        double xDistanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
            this.getCameraConfig().transform.getMeasureZ().in(Meters), FieldConstants.fuelRadiusMeters,
            -this.getCameraConfig().transform.getRotation().getMeasureY().in(Radians),
            Units.degreesToRadians(target.getPitch()));

        double distanceToTargetMeters = xDistanceMeters / Math.cos(Units.degreesToRadians(target.getYaw()));

        Translation2d cameraToObject = PhotonUtils.estimateCameraToTargetTranslation(distanceToTargetMeters,
            new Rotation2d(-Units.degreesToRadians(target.getYaw())));

        Pose3d objectPose = new Pose3d(
            robotPose.plus(ObjectDetectionUtils.transform3dTo2d(this.getCameraConfig().transform))
                .plus(new Transform2d(cameraToObject, Rotation2d.kZero)))
                    .plus(new Transform3d(0, 0, FieldConstants.fuelRadiusMeters, Rotation3d.kZero));
        objectPoses.add(objectPose);
        photonObjects.add(new PhotonObjectDetected(objectPose, target.getDetectedObjectClassID(),
            target.getDetectedObjectConfidence(), target.getPitch(), target.getYaw(), target.getSkew(),
            target.getArea()));
      }
    }
    return photonObjects;

  }
}
