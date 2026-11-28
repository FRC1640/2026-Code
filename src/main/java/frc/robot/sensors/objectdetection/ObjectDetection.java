package frc.robot.sensors.objectdetection;

import org.littletonrobotics.junction.Logger;

import frc.robot.util.periodic.PeriodicBase;

public class ObjectDetection extends PeriodicBase {
  ObjectDetectionVisionIO io;
  ObjectDetectionIOInputsAutoLogged inputs;
  public ObjectDetection(ObjectDetectionVisionIO io) {
    this.io = io;
    inputs = new ObjectDetectionIOInputsAutoLogged();
  }
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ObjectDetection", inputs);
  }

  public PhotonObjectDetected[] getObjectDetecteds() {
    return inputs.objectsDetected;
  }
}
