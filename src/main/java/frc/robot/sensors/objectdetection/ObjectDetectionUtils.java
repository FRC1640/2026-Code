package frc.robot.sensors.objectdetection;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.sensors.objectdetection.cameras.ODCameraPhotonReal;
import frc.robot.sensors.objectdetection.cameras.ODCameraPhotonSim;
import frc.robot.sensors.objectdetection.cameras.ObjectDetectionCameraConstant;

public class ObjectDetectionUtils {
  public static Transform2d transform3dTo2d(Transform3d other) {
    return new Transform2d(other.getTranslation().toTranslation2d(), other.getRotation().toRotation2d());
  }
  public static List<ODCameraPhotonReal> getPhotonRealCamerasFromConstants(
      List<ObjectDetectionCameraConstant> cameraConstants) {
    List<ODCameraPhotonReal> cameras = new ArrayList<>();
    cameraConstants.forEach((x) -> cameras.add(new ODCameraPhotonReal(x)));
    return cameras;
  }

  public static List<ODCameraPhotonSim> getPhotonSimCamerasFromConstants(
      List<ObjectDetectionCameraConstant> cameraConstants) {
    List<ODCameraPhotonSim> cameras = new ArrayList<>();
    cameraConstants.forEach((x) -> cameras.add(new ODCameraPhotonSim(x)));
    return cameras;
  }
}
