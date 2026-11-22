package frc.robot.sensors.objectdetection;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.sensors.objectdetection.cameras.ObjectDetectionCameraConstant;

public interface ObjectDetectionVisionIO {
  @AutoLog
  public class ObjectDetectionIOInputs {
    public PhotonObjectDetected[] objectsDetected = new PhotonObjectDetected[0];
  }

  public default void updateInputs(ObjectDetectionIOInputs inputs) {
  };

  public static ObjectDetectionVisionIO getObjectDetectionIO(List<ObjectDetectionCameraConstant> constants,
      Supplier<Pose2d> robotPose) {
    return switch (Robot.getMode()) {
      case REAL -> new ObjectDetectionVisionIOPhoton(
          ObjectDetectionUtils.getPhotonRealCamerasFromConstants(constants), robotPose);
      case SIM -> new ObjectDetectionVisionIOSim(ObjectDetectionUtils.getPhotonSimCamerasFromConstants(constants),
          robotPose);
      case REPLAY -> new ObjectDetectionVisionIO() {

      };
    };
  }
  public static ObjectDetectionVisionIO getObjectDetectionRealIO(List<ObjectDetectionCameraConstant> constants,
      Supplier<Pose2d> robotPose) {
    return new ObjectDetectionVisionIOPhoton(
          ObjectDetectionUtils.getPhotonRealCamerasFromConstants(constants), robotPose);
  }
}
