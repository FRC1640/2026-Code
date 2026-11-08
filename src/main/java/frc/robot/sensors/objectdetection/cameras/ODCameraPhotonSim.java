package frc.robot.sensors.objectdetection.cameras;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class ODCameraPhotonSim implements ODCameraPhoton {
  ObjectDetectionCameraConstant cameraConstant;
  PhotonCamera photonCamera;
  PhotonCameraSim photonCameraSim;
  public ODCameraPhotonSim(ObjectDetectionCameraConstant cameraConstant) {
    this.cameraConstant = cameraConstant;
    this.photonCamera = new PhotonCamera(cameraConstant.networkName);
    this.photonCameraSim = new PhotonCameraSim(photonCamera, cameraConstant.cameraProperties);
  }
  @Override
  public List<PhotonPipelineResult> getResults() {
    return photonCamera.getAllUnreadResults();
  }

  @Override
  public ObjectDetectionCameraConstant getCameraConfig() {
    return cameraConstant;
  }

  public PhotonCameraSim getPhotonCameraSim() {
    return photonCameraSim;
  }
}
