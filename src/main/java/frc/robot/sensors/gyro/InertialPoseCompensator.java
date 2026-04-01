package frc.robot.sensors.gyro;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.periodic.PeriodicBase;

public class InertialPoseCompensator extends PeriodicBase {
  private Gyro gyro;

  private Pose2d referencePose = new Pose2d();
  private ChassisSpeeds referenceSpeeds = new ChassisSpeeds();

  private Pose2d estimatedPose = new Pose2d();
  private Translation2d estimatedLinearSpeeds = new Translation2d();

  public InertialPoseCompensator(Gyro gyro) {
    this.gyro = gyro;
  }

  public void beginTrack(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    referencePose = currentPose;
    referenceSpeeds = currentSpeeds;
    estimatedPose = referencePose;
    estimatedLinearSpeeds = new Translation2d(referenceSpeeds.vxMetersPerSecond, referenceSpeeds.vyMetersPerSecond);
  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  @Override
  public void periodic() {
    double accelX = gyro.getAccelX();
    double accelY = gyro.getAccelY();
    Translation2d linearAcceleration = new Translation2d(accelX, accelY).rotateBy(gyro.getAngleRotation2d());
    estimatedLinearSpeeds = estimatedLinearSpeeds.plus(linearAcceleration.times(0.02));
    Translation2d poseTranslation = estimatedPose.getTranslation().plus(estimatedLinearSpeeds.times(0.02));
    estimatedPose = new Pose2d(poseTranslation, gyro.getAngleRotation2d());

    Logger.recordOutput("PoseCompensation/linearAcceleration", linearAcceleration);
    Logger.recordOutput("PoseCompensation/linearVelocity", estimatedLinearSpeeds);
    Logger.recordOutput("PoseCompensation/pose", estimatedPose);
  }
}
