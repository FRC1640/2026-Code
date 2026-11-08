package frc.robot.sensors.objectdetection;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.sensors.objectdetection.cameras.ODCameraPhotonSim;

public class ObjectDetectionVisionIOSim implements ObjectDetectionVisionIO {

  List<ODCameraPhotonSim> cameras;
  Supplier<Pose2d> robotSupplier;
  VisionSystemSim visionSim = new VisionSystemSim("object_detection");

  public ObjectDetectionVisionIOSim(List<ODCameraPhotonSim> cameras, Supplier<Pose2d> robotSupplier) {
    this.cameras = cameras;
    this.robotSupplier = robotSupplier;
    cameras.forEach(x -> visionSim.addCamera(x.getPhotonCameraSim(), x.getCameraConfig().transform));
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    cameras.forEach(x -> x.update(robotSupplier.get()));
  }
}
