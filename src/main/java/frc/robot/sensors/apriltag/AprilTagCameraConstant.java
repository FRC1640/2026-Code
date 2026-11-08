package frc.robot.sensors.apriltag;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.sensors.CameraConstant;

public class AprilTagCameraConstant extends CameraConstant {
  public final double standardDevConstant;

  public AprilTagCameraConstant(SimCameraProperties cameraProperties, Transform3d transform,
      double standardDevConstant, String networkName, String displayName) {
    super(cameraProperties, transform, networkName, displayName);
    this.standardDevConstant = standardDevConstant;
  }
}
