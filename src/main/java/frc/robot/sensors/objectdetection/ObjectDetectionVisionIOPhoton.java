package frc.robot.sensors.objectdetection;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.sensors.objectdetection.cameras.ODCameraPhotonReal;

public class ObjectDetectionVisionIOPhoton implements ObjectDetectionVisionIO {

  List<ODCameraPhotonReal> cameras;
  Supplier<Pose2d> robotSupplier;
  public ObjectDetectionVisionIOPhoton(List<ODCameraPhotonReal> cameras, Supplier<Pose2d> robotSupplier) {
    this.cameras = cameras;
    this.robotSupplier = robotSupplier;
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    cameras.forEach(x -> x.update(robotSupplier.get()));
  }
}
