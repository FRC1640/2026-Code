package frc.robot.sensors.objectdetection;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionVisionIO {
    @AutoLog
    public class ObjectDetectionIOInputs {
        public PhotonObjectDetected[] objectsDetected = new PhotonObjectDetected[0];

    }

    public void updateInputs(ObjectDetectionIOInputs inputs);
}
