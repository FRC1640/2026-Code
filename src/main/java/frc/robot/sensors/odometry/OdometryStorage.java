package frc.robot.sensors.odometry;

import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.RobotDimensions;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.odometry.RobotOdometry.VisionUpdateMode;
import frc.robot.util.helpers.DistanceManager;

public class OdometryStorage {
  private SwerveDrivePoseEstimator estimator;

  private AprilTagVision[] visions;
  private VisionUpdateMode updateMode;

  private Pose2d lastPose = new Pose2d();
  private ChassisSpeeds estimatedVelocity = new ChassisSpeeds();

  private boolean driveUntrustworthy = false;
  private double visionStdDevCompensation = 1;
  private final double trustResetDistanceThreshold = 0.04;

  private boolean clampPoseInField = false;
  private DoubleSupplier clampingRotation = null;

  private OdometryStorage trustedRotation = null;
  public Rotation2d rawGyroRotation = new Rotation2d();
  private final double gyroBufferSizeSec = 2.0;

  public TimeInterpolatableBuffer<Rotation2d> gyroBuffer = TimeInterpolatableBuffer.createBuffer((a, b, c) -> {
    double aRadians = a.getRadians();
    double delta = (b.getRadians() % (2 * Math.PI)) - aRadians;
    return new Rotation2d((delta * c) + aRadians);
  }, gyroBufferSizeSec);

  public SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(),
          new SwerveModulePosition(), new SwerveModulePosition()};

  private String name;

  public OdometryStorage(String name, SwerveDrivePoseEstimator estimator, AprilTagVision[] visions,
      VisionUpdateMode updateMode) {
    this.estimator = estimator;
    this.visions = visions;
    this.updateMode = updateMode;
    this.name = name;
  }

  public OdometryStorage(String name, SwerveDrivePoseEstimator estimator, AprilTagVision[] visions,
      VisionUpdateMode updateMode, OdometryStorage trustedRotation) {
    this.estimator = estimator;
    this.visions = visions;
    this.updateMode = updateMode;
    this.name = name;
    this.trustedRotation = trustedRotation;
  }

  public Pose2d getEstimatedPosition() {
    return estimator.getEstimatedPosition();
  }

  public ChassisSpeeds getEstimatedVelocity() {
    return estimatedVelocity;
  }

  public void updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
    estimator.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);
    if (clampPoseInField) {
      estimator.resetPose(clampPose(estimator.getEstimatedPosition()));
    }
  }

  public void addVisionMeasurement(Pose2d measurement, double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    if (driveUntrustworthy) {
      estimator.addVisionMeasurement(measurement, timestampSeconds,
          visionMeasurementStdDevs.times(visionStdDevCompensation));
      if (estimator.getEstimatedPosition().minus(measurement).getTranslation()
          .getNorm() < trustResetDistanceThreshold) {
        driveUntrustworthy = false;
      }
      return;
    }
    estimator.addVisionMeasurement(measurement, timestampSeconds, visionMeasurementStdDevs);
    if (clampPoseInField) {
      estimator.resetPose(clampPose(estimator.getEstimatedPosition()));
    }
  }

  public void updatePoseVelocity() {
    Pose2d pose = getEstimatedPosition();
    Translation2d linearVelocity = pose.getTranslation().minus(lastPose.getTranslation()).div(0.02);
    estimatedVelocity.vxMetersPerSecond = linearVelocity.getX();
    estimatedVelocity.vyMetersPerSecond = linearVelocity.getY();
    estimatedVelocity.omegaRadiansPerSecond = pose.getRotation().minus(lastPose.getRotation()).getRadians() / 0.02;
    this.lastPose = getEstimatedPosition();
    Logger.recordOutput("Drive/Odometry/" + name + "/estimatedVelocity", estimatedVelocity);
  }

  public void resetPose(Pose2d pose) {
    estimator.resetPose(pose);
  }

  public void resetTranslation(Translation2d translation) {
    estimator.resetTranslation(translation);
  }

  public void resetRotation(Rotation2d rotation) {
    estimator.resetRotation(rotation);
  }

  public void distrustDrive() {
    driveUntrustworthy = true;
  }

  public boolean isDriveUntrustworthy() {
    return driveUntrustworthy;
  }

  public OdometryStorage setVisionStdDevCompensation(double factor) {
    visionStdDevCompensation = factor;
    return this;
  }

  public OdometryStorage setClampPoseInField(boolean enable) {
    this.clampPoseInField = enable;
    return this;
  }

  public OdometryStorage setClampingRotation(DoubleSupplier rotation) {
    this.clampingRotation = rotation;
    return this;
  }

  private Pose2d clampPose(Pose2d pose) {
    if (!requiresClamping(pose))
      return pose;
    Rotation2d angle = new Rotation2d(
        clampingRotation == null ? pose.getRotation().getRadians() : clampingRotation.getAsDouble());
    Translation2d[] robotCorners = new Translation2d[4];
    for (int i = 0; i < 4; i++) {
      robotCorners[i] = RobotDimensions.robotCorners[i].rotateBy(angle);
    }
    double[] distances = new double[4];
    for (int i = 0; i < 4; i++) {
      Translation2d direction = new Translation2d(Math.cos(i * Math.PI / 2), Math.sin(i * Math.PI / 2));
      distances[i] = DistanceManager.calculatePolygonProtrusion(robotCorners, direction);
    }
    return new Pose2d(
        new Translation2d(MathUtil.clamp(pose.getX(), distances[2], FieldConstants.fieldWidth - distances[0]),
            MathUtil.clamp(pose.getY(), distances[3], FieldConstants.fieldHeight - distances[1])),
        angle);
  }

  private boolean requiresClamping(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double padding = RobotDimensions.robotBoundingSquareEdge / Math.sqrt(2);
    boolean clampingRequired = !(x > padding && x < FieldConstants.fieldWidth - padding && y > padding
        && y < FieldConstants.fieldHeight - padding);
    Logger.recordOutput("Drive/Odometry/" + name + "/poseClampingRequired", clampingRequired);
    return clampingRequired;
  }

  public OdometryStorage setTrustedRotation(OdometryStorage trustedRotation) {
    this.trustedRotation = trustedRotation;
    return this;
  }

  public Optional<OdometryStorage> getTrustedRotation() {
    return Optional.ofNullable(trustedRotation);
  }

  public void addGyroSample(Rotation2d sample, double timestamp) {
    gyroBuffer.addSample(timestamp, sample);
  }

  public Optional<Rotation2d> getGyroAtTimestamp(double timestamp) {
    try {
      if (gyroBuffer.getInternalBuffer().lastKey() - gyroBufferSizeSec > timestamp) {
        return Optional.empty();
      }
    } catch (NoSuchElementException e) {
      return Optional.empty();
    }
    return gyroBuffer.getSample(timestamp);
  }

  public AprilTagVision[] getVisions() {
    return visions;
  }

  public VisionUpdateMode getUpdateMode() {
    return updateMode;
  }

  public String getName() {
    return name;
  }

  @Override
  public int hashCode() {
    return estimator.hashCode();
  }
}
