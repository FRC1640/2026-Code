package frc.robot.sensors.objectdetection.cameras;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class ODCameraPhotonReal implements ODCameraPhoton {
  ObjectDetectionCameraConstant cameraConstant;
  PhotonCamera photonCamera;
  public ODCameraPhotonReal(ObjectDetectionCameraConstant cameraConstant) {
    this.cameraConstant = cameraConstant;
    this.photonCamera = new PhotonCamera(cameraConstant.networkName);
  }
  @Override
  public List<PhotonPipelineResult> getResults() {
    return photonCamera.getAllUnreadResults();
  }

  @Override
  public ObjectDetectionCameraConstant getCameraConfig() {
    return cameraConstant;
  }

}
