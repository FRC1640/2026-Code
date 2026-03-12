package frc.robot.sensors.odometry;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.CameraConstants;
import frc.robot.constants.RobotConstants.RobotState;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.apriltag.AprilTagVisionIO.PoseObservation;
import frc.robot.sensors.gyro.BumpDetectorPeriodic;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.periodic.PeriodicBase;

public class RobotOdometry extends PeriodicBase {
  public enum VisionUpdateMode {
    PHOTONVISION, TRIG
  }

  private final DriveSubsystem driveSubsystem;
  private final Gyro gyro;

  private BumpDetectorPeriodic bumpDetector = null;

  private final HashMap<String, OdometryStorage> odometries = new HashMap<>();
  private final HashMap<String, AprilTagVision> visionMap = new HashMap<>();

  private boolean useAutoApriltags = true;

  public static RobotOdometry instance;

  public RobotOdometry(DriveSubsystem driveSubsystem, Gyro gyro, AprilTagVision... cameras) {
    instance = this;
    this.driveSubsystem = driveSubsystem;
    this.gyro = gyro;
    for (AprilTagVision aprilTagVision : cameras) {
      visionMap.put(aprilTagVision.getDisplayName(), aprilTagVision);
    }
    SparkOdometryThread.getInstance().start();
    branchEstimator("Main", cameras, VisionUpdateMode.PHOTONVISION)
        .setVisionStdDevCompensation(CameraConstants.bumpVisionStdDevFactor);
  }

  public void setBumpDetector(BumpDetectorPeriodic bumpDetector) {
    this.bumpDetector = bumpDetector;
  }

  public boolean usingAutoApriltags() {
    return useAutoApriltags;
  }

  public void setAutoApriltags(boolean useAutoApriltags) {
    this.useAutoApriltags = useAutoApriltags;
  }

  public void resetGyro(Rotation2d newRotation) {
    gyro.setOffset(gyro.getRawAngleRadians() - newRotation.getRadians()
        + (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
            ? Math.PI
            : 0));
  }

  public Command resetGyroCommand(Supplier<Rotation2d> newRotation) {
    return new InstantCommand(() -> resetGyro(newRotation.get()));
  }

  /*---------------------
  | ESTIMATOR UTILITIES |
  ---------------------*/

  public static SwerveDrivePoseEstimator getDefaultEstimator(Pose2d initalPose) {
    return new SwerveDrivePoseEstimator(DriveConstants.kinematics, new Rotation2d(),
        new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()},
        new Pose2d(), CameraConstants.defaultDriveStandardDev, CameraConstants.defaultVisionStandardDev);
  }

  public OdometryStorage branchEstimator(String name, String[] cameras, VisionUpdateMode visionUpdateMode) {
    OdometryStorage o = new OdometryStorage(name, getDefaultEstimator(new Pose2d()),
        Arrays.stream(cameras).map((x) -> visionMap.get(x)).toArray(AprilTagVision[]::new), visionUpdateMode);
    odometries.put(name, o);
    return o;
  }

  public OdometryStorage branchEstimator(String name, AprilTagVision[] cameras, VisionUpdateMode visionUpdateMode) {
    OdometryStorage o = new OdometryStorage(name, getDefaultEstimator(new Pose2d()), cameras, visionUpdateMode);
    odometries.put(name, o);
    return o;
  }

  public OdometryStorage branchEstimator(String name, AprilTagVision[] cameras, VisionUpdateMode visionUpdateMode,
      OdometryStorage branchFrom) {
    OdometryStorage o = new OdometryStorage(name, getDefaultEstimator(branchFrom.getEstimatedPosition()), cameras,
        visionUpdateMode);
    odometries.put(name, o);
    return o;
  }

  public OdometryStorage branchEstimator(String name, String[] cameras, VisionUpdateMode visionUpdateMode,
      OdometryStorage branchFrom) {
    OdometryStorage o = new OdometryStorage(name, getDefaultEstimator(branchFrom.getEstimatedPosition()),
        Arrays.stream(cameras).map((x) -> visionMap.get(x)).toArray(AprilTagVision[]::new), visionUpdateMode);
    odometries.put(name, o);
    return o;
  }

  public void pruneBranch(OdometryStorage estimator) {
    odometries.remove(estimator.getName());
  }

  public void setVisionStdDevFactor(String name, double factor) {
    odometries.get(name).setVisionStdDevCompensation(factor);
  }

  public void resetVisionStdDevFactor(String name) {
    setVisionStdDevFactor(name, 1);
  }

  public void distrustDrive(String name) {
    odometries.get(name).distrustDrive();
  }

  public boolean isDriveUntrustworthy(String name) {
    return odometries.get(name).isDriveUntrustworthy();
  }

  /*------------------
  | ODOMETRY UPDATES |
  ------------------*/

  public void updateAllOdometries() {
    for (var estimator : odometries.values()) {
      updateSwerveOdometry(estimator);
      Logger.recordOutput("Drive/Odometry/" + estimator.getName() + "/driveUntrustworthy",
          estimator.isDriveUntrustworthy());

      for (AprilTagVision aprilTagVision : estimator.getVisions()) {
        switch (estimator.getUpdateMode()) {
          case PHOTONVISION :
            addPhotonEstimate(estimator, aprilTagVision);
            break;

          case TRIG :
            addTrigEstimate(estimator, aprilTagVision);
            break;
        }
      }
      estimator.updatePoseVelocity();
    }
  }

  public Pose2d getPose(String name) {
    return odometries.get(name).getEstimatedPosition();
  }

  public ChassisSpeeds getVelocity(String name) {
    return odometries.get(name).getEstimatedVelocity();
  }

  public void setPose(String name, Pose2d pose) {
    odometries.get(name).resetPose(pose);
  }

  public void setPoseNoRot(String name, Pose2d pose) {
    odometries.get(name).resetTranslation(pose.getTranslation());
  }

  public void setPoseRot(String name, Pose2d pose) {
    odometries.get(name).resetRotation(pose.getRotation());
  }

  public void setAllPose(Pose2d pose) {
    for (OdometryStorage odometryStorage : odometries.values()) {
      odometryStorage.resetPose(pose);
    }
  }

  public void addPhotonEstimate(OdometryStorage odometry, AprilTagVision vision) {
    if (Robot.isSimulation())
      return;

    List<Pose2d> robotPoses = new LinkedList<>();
    List<Pose2d> robotPosesAccepted = new LinkedList<>();
    List<Pose2d> robotPosesRejected = new LinkedList<>();

    for (PoseObservation poseObservation : vision.getPhotonResults()) {
      // validity checks
      Pose2d visionUpdate = poseObservation.pose().toPose2d();
      robotPoses.add(visionUpdate);
      if (!(isPhotonEstimateValid(poseObservation, vision.getRotationValidPhotonObservation(poseObservation))
          && vision.isConnected())) {
        robotPosesRejected.add(visionUpdate);
        continue;
      }
      robotPosesAccepted.add(visionUpdate);

      // add measurement
      double xy = vision.getPhotonXyStdDev(poseObservation);
      double rot = vision.getPhotonRotStdDev(poseObservation);
      odometry.addVisionMeasurement(visionUpdate, poseObservation.timestamp(), VecBuilder.fill(xy, xy, rot));
    }
    for (Pose2d pose : robotPoses) {
      Logger.recordOutput("Sensors/AprilTagVision/" + vision.getDisplayName() + "/RobotPoses", pose);
    }
    for (Pose2d pose : robotPosesAccepted) {
      Logger.recordOutput("Sensors/AprilTagVision/" + vision.getDisplayName() + "/RobotPosesAccepted", pose);
    }
    for (Pose2d pose : robotPosesRejected) {
      Logger.recordOutput("Sensors/AprilTagVision/" + vision.getDisplayName() + "/RobotPosesRejected", pose);
    }
  }

  private boolean isPhotonEstimateValid(PoseObservation observation, boolean rotationValid) {
    Pose2d visionUpdate = observation.pose().toPose2d();
    return Robot.getState() != RobotState.DISABLED
        && (Robot.getState() != RobotState.AUTONOMOUS || useAutoApriltags) && isPoseValid(visionUpdate)
        && observation.tagCount() > 0 && observation.ambiguity() < 0.2 && observation.minimumTagDistance() < 7
        && Math.abs(observation.pose().getZ()) < 0.75
        && (Math.abs(observation.pose().getRotation().toRotation2d()
            .minus(RobotOdometry.instance.getPose("Main").getRotation()).getDegrees()) < 1 || rotationValid)
        && (bumpDetector == null || !bumpDetector.bumpDetected());
  }

  public boolean isPoseValid(Pose2d pose) {
    return FieldConstants.fieldWidth >= pose.getX() && FieldConstants.fieldHeight >= pose.getY() && pose.getX() > 0
        && pose.getY() > 0;
  }

  public void addTrigEstimate(OdometryStorage odometry, AprilTagVision vision) {
    if (odometry.getTrustedRotation().isEmpty())
      return;
    if (vision.getTrigResult(new Rotation2d()).isEmpty())
      return;

    Optional<Rotation2d> interpolateGyro = odometry.getTrustedRotation().get()
        .getGyroAtTimestamp(vision.getTrigResult(new Rotation2d()).get().timestamp());

    if (interpolateGyro.isEmpty())
      return;

    // calculate estimated pose (trig)
    Optional<PoseObservation> result = vision
        .getTrigResult(odometry.getTrustedRotation().get().getEstimatedPosition().getRotation());

    // return if no result; continue otherwise
    if (result.isEmpty()) {
      return;
    }
    result = vision
        .getTrigResult(odometry.getTrustedRotation().get().getGyroAtTimestamp(result.get().timestamp()).get());
    Pose2d visionUpdate = result.get().pose().toPose2d();
    Logger.recordOutput("Sensors/AprilTagVision/" + vision.getDisplayName() + "/RobotPosesTrig", visionUpdate);
    if (Robot.getState() == RobotState.DISABLED
        || (Robot.getState() == RobotState.AUTONOMOUS && !useAutoApriltags)) {
      Logger.recordOutput("Sensors/AprilTagVision/" + vision.getDisplayName() + "/RobotPosesRejectedTrig",
          visionUpdate);
      return;
    }
    if (!(isPoseValid(visionUpdate) && vision.isConnected() && result.get().minimumTagDistance() < 7)) {
      Logger.recordOutput("AprilTagVision/" + vision.getDisplayName() + "/RobotPosesRejectedTrig", visionUpdate);
      return;
    }
    if (bumpDetector != null && bumpDetector.bumpDetected()) {
      Logger.recordOutput("AprilTagVision/" + vision.getDisplayName() + "/RobotPosesRejectedTrig", visionUpdate);
      return;
    }
    Logger.recordOutput("AprilTagVision/" + vision.getDisplayName() + "/RobotPosesAcceptedTrig", visionUpdate);
    double xy = vision.getTrigXyStdDev(result.get());
    odometry.addVisionMeasurement(visionUpdate, result.get().timestamp(), VecBuilder.fill(xy, xy, 0.00000001));
  }

  public void updateSwerveOdometry(OdometryStorage odometry) {
    double[] sampleTimestamps = driveSubsystem.getModules()[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = driveSubsystem.getModules()[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
            modulePositions[moduleIndex].distanceMeters
                - odometry.lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle);
        odometry.lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }
      // Update gyro angle
      if (gyro.isTrustworthy()) {
        // Use the real gyro angle
        Rotation2d update = gyro.getOdometryPositions()[i];
        odometry.rawGyroRotation = update;
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = DriveConstants.kinematics.toTwist2d(moduleDeltas);
        Rotation2d update = odometry.rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        odometry.rawGyroRotation = update;
      }

      // apply update
      odometry.updateWithTime(sampleTimestamps[i], odometry.rawGyroRotation, modulePositions);
      odometry.addGyroSample(odometry.getEstimatedPosition().getRotation(), sampleTimestamps[i]);
      Logger.recordOutput("Drive/Odometry/" + odometry.getName(), odometry.getEstimatedPosition());
    }
  }

  @Override
  public void periodic() {
    updateAllOdometries();
    Logger.recordOutput("gyroBufferSize",
        RobotOdometry.instance.odometries.get("Main").gyroBuffer.getInternalBuffer().size());
  }
}
