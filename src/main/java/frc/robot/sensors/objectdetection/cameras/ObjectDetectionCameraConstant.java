package frc.robot.sensors.objectdetection.cameras;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.sensors.CameraConstant;

public class ObjectDetectionCameraConstant extends CameraConstant {
  public ObjectDetectionCameraConstant(SimCameraProperties cameraProperties, Transform3d transform,
      String networkName, String displayName) {
    super(cameraProperties, transform, networkName, displayName);
  }
}
