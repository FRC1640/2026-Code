package frc.robot.sensors.objectdetection;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;
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
import frc.robot.sensors.objectdetection.ObjectDetectionVisionIO.ObjectDetectionIOInputs;

public class ObjectDetectionVisionIOPhoton implements ObjectDetectionVisionIO {

    PhotonCamera camera; 
    ObjectDetectionCameraConstant constant;
    Supplier<Pose2d> robotSupplier;
    public ObjectDetectionVisionIOPhoton(ObjectDetectionCameraConstant constant, Supplier<Pose2d> robotSupplier) {
        this.constant = constant;
        camera = new PhotonCamera(constant.networkName);
        this.robotSupplier = robotSupplier;
    }
    
    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        List<PhotonPipelineResult> photonResults = camera.getAllUnreadResults();
        List<PhotonTrackedTarget> targets = new ArrayList<>();
        List<Pose3d> objectPoses = new ArrayList<>();
        List<PhotonObjectDetected> photonObjects = new ArrayList<>();
        
        for (PhotonPipelineResult result : photonResults) {
            for (int i = 0; i < result.getTargets().size(); i++) {
            PhotonTrackedTarget target = result.getTargets().get(i);
            targets.add(target);
               double xDistanceMeters =
              PhotonUtils.calculateDistanceToTargetMeters(
                  this.constant.transform.getMeasureZ().in(Meters),
                  FieldConstants.fuelRadiusMeters,
                  -this.constant.transform.getRotation().getMeasureY().in(Radians),
                  Units.degreesToRadians(target.getPitch()));

          double distanceToTargetMeters =
              xDistanceMeters / Math.cos(Units.degreesToRadians(target.getYaw()));

          Translation2d cameraToObject =
              PhotonUtils.estimateCameraToTargetTranslation(
                  distanceToTargetMeters, new Rotation2d(-Units.degreesToRadians(target.getYaw())));

          Pose3d objectPose =
              new Pose3d(
                      this.robotSupplier
                          .get()
                          .plus(transform3dTo2d(this.constant.transform))
                          .plus(new Transform2d(cameraToObject, Rotation2d.kZero)))
                  .plus(new Transform3d(0, 0, FieldConstants.fuelRadiusMeters, Rotation3d.kZero));
                objectPoses.add(objectPose);
          photonObjects.add(new PhotonObjectDetected(objectPose, target.getDetectedObjectClassID(), target.getPoseAmbiguity(), target.getPitch(), target.getYaw(), target.getSkew(), target.getArea()));
            }
        }
        inputs.objectsDetected = photonObjects.toArray(new PhotonObjectDetected[0]);
    }
    
  public static Transform2d transform3dTo2d(Transform3d other) {
    return new Transform2d(
        other.getTranslation().toTranslation2d(), other.getRotation().toRotation2d());
  }

}
