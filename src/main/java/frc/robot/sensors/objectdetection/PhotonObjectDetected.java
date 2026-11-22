package frc.robot.sensors.objectdetection;

import edu.wpi.first.math.geometry.Pose3d;

public record PhotonObjectDetected(Pose3d pose, int classId, double confidence, double pitch, double yaw, double skew,
    double area, Pose3d translationRelativeToRobot) {

}
