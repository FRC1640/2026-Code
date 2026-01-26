package frc.robot.sensors.odometry;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.apriltag.AprilTagVisionIO.PoseObservation;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.periodic.PeriodicBase;

public class BumpOdometry extends PeriodicBase {
  private final DriveSubsystem driveSubsystem;
  private final Gyro gyro;

  private final List<AprilTagVision> visions = new ArrayList<>();

  private static BumpOdometry instance;

  public static BumpOdometry getInstance() {
    return instance;
  }

  public BumpOdometry(DriveSubsystem driveSubsystem, Gyro gyro, AprilTagVision... visions) {
    BumpOdometry.instance = this;
    this.driveSubsystem = driveSubsystem;
    this.gyro = gyro;
    for (AprilTagVision vision : visions) {
      this.visions.add(vision);
    }
  }

  private SwerveDrivePoseEstimator poseEstimator;

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  private void updateOdometry() {
    updateWheelOdometry();
    for (AprilTagVision vision : visions) {
      addVisionMeasurement(vision);
    }
  }

  private void updateWheelOdometry() {

  }

  private void addVisionMeasurement(AprilTagVision vision) {
    if (Robot.isSimulation())
      return;

    List<Pose2d> robotPoses = new LinkedList<>();
    List<Pose2d> robotPosesAccepted = new LinkedList<>();
    List<Pose2d> robotPosesRejected = new LinkedList<>();

    for (PoseObservation observation : vision.getPhotonResults()) {

    }
  }
}
