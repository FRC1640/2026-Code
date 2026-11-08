package frc.robot.sensors;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.SimCameraProperties;

public class CameraConstant {
  public final SimCameraProperties cameraProperties;
  public final Transform3d transform;
  public final String networkName;
  public final String displayName;

  public CameraConstant(SimCameraProperties cameraProperties, Transform3d transform,
      String networkName, String displayName) {
    this.cameraProperties = cameraProperties;
    this.transform = transform;
    this.networkName = networkName;
    this.displayName = displayName;
  }
}
