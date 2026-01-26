package frc.robot.sensors.odometry;

import java.util.NoSuchElementException;
import java.util.Optional;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.odometry.RobotOdometry.VisionUpdateMode;

public class OdometryStorage {
  public SwerveDrivePoseEstimator estimator;

  private AprilTagVision[] visions;
  private VisionUpdateMode updateMode;

  public Rotation2d rawGyroRotation = new Rotation2d();
  private OdometryStorage trustedRotation = null;

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

  public void setTrustedRotation(OdometryStorage trustedRotation) {
    this.trustedRotation = trustedRotation;
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
