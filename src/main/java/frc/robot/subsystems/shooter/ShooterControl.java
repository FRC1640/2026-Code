package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.turret.TurretConstants.turretZeroOffsetRobotFrame;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.util.helpers.AllianceManager;
import frc.robot.util.helpers.DistanceManager;

public class ShooterControl {
  private static HashMap<Integer, Translation2d> hubTags = new HashMap<>();

  private Supplier<Pose2d> robotPose;
  private Supplier<ChassisSpeeds> robotVelocity;
  private Supplier<Pose2d> targetPose;
  private Supplier<Rotation2d> robotRotation;
  private AprilTagVision turretCamera;
  private static DoubleSupplier turretAngle;

  private static ShooterControl instance;

  public static record TurretSetpoint(double turretAngle, double turretOmega, double hoodAngle,
      double flywheelSpeed) {
  }

  private TurretSetpoint setpoint;
  private TurretSetpoint lastSetpoint;

  private static final InterpolatingDoubleTreeMap distanceToDeflectorAngle = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToFlywheelSpeed = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToTimeOfFlight = new InterpolatingDoubleTreeMap();

  static {
    // TODO initialize lookup tables
  }

  public ShooterControl(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotVelocity,
      Supplier<Pose2d> targetPose, Supplier<Rotation2d> robotRotation, AprilTagVision turretCamera) {
    this.robotPose = robotPose;
    this.robotVelocity = robotVelocity;
    this.targetPose = targetPose;
    this.robotRotation = robotRotation;
    this.turretCamera = turretCamera;
    /*
     * hubTags.put(AllianceManager.chooseFromAlliance(25, 9),
     * AllianceManager.chooseFromAlliance( FieldConstants.hubPositionBlue
     * .minus(FieldConstants.aprilTagLayout.getTagPose(25).get().toPose2d()).
     * getTranslation(), FieldConstants.hubPositionRed
     * .minus(FieldConstants.aprilTagLayout.getTagPose(9).get().toPose2d()).
     * getTranslation())); hubTags.put(AllianceManager.chooseFromAlliance(26, 10),
     * AllianceManager.chooseFromAlliance(
     * FieldConstants.hubPositionBlue.minus(FieldConstants.aprilTagLayout.getTagPose
     * (26).get().toPose2d()) .getTranslation(),
     * FieldConstants.hubPositionRed.minus(FieldConstants.aprilTagLayout.getTagPose(
     * 10).get().toPose2d()) .getTranslation())); for (int id : hubTags.keySet()) {
     * turretCamera.addTrackingId(id); }
     */
    setpoint = new TurretSetpoint(0, 0, 0, 0);
    lastSetpoint = new TurretSetpoint(0, 0, 0, 0);
    ShooterControl.instance = this;
  }

  public static ShooterControl getInstance() {
    return instance;
  }

  public static void clearSetpoint() {
    if (instance.setpoint != null)
      instance.lastSetpoint = instance.setpoint;
    instance.setpoint = null;
  }

  public static void setTurretAngleSupplier(DoubleSupplier turretAngle) {
    ShooterControl.turretAngle = turretAngle;
  }

  public TurretSetpoint getSetpointGlobal() {
    if (setpoint != null) {
      return setpoint;
    }

    ChassisSpeeds velocity = robotVelocity.get();
    Pose2d turretPose = robotPose.get().plus(TurretConstants.turretTransform2d);
    // calculate turret velocity
    Translation2d turretVelocity = turretPose.getTranslation().minus(robotPose.get().getTranslation())
        .rotateBy(Rotation2d.kCCW_Pi_2).times(velocity.omegaRadiansPerSecond)
        .plus(new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond));
    // calculate distance to target
    Translation2d targetOffset = targetPose.get().getTranslation().minus(turretPose.getTranslation());
    // calculate distance to adjusted target accounting for robot velocity
    Translation2d deltaR = new Translation2d();// turretVelocity.times(distanceToTimeOfFlight.get(targetOffset.getNorm()));
    Translation2d adjustedDistance = targetOffset.minus(deltaR);

    // use lookup tables to get hood angle and flywheel speed
    // double flywheelSpeed =
    // distanceToFlywheelSpeed.get(adjustedDistance.getNorm());
    // double deflectorAngle =
    // distanceToDeflectorAngle.get(adjustedDistance.getNorm());

    // calculate turret angle setpoint
    double turretAngle = targetOffset.getNorm() != 0
        ? targetOffset.getAngle()
            .minus(robotPose.get().getRotation()
                .plus(new Rotation2d(TurretConstants.turretZeroOffsetRobotFrame)))
            .getRadians()
        : 0;

    TurretSetpoint output = new TurretSetpoint(turretAngle, (turretAngle - lastSetpoint.turretAngle()) / 0.02,
        /* deflectorAngle */0, /* flywheelSpeed */0);

    lastSetpoint = setpoint;
    setpoint = output;

    Logger.recordOutput("Shooter/setpoint", setpoint);
    Logger.recordOutput("Shooter/turretPose", turretPose);
    Logger.recordOutput("Shooter/targetOffset", targetOffset);
    Logger.recordOutput("Shooter/turretTargeting", robotPose.get()
        .plus(new Transform2d(new Translation2d(1, new Rotation2d(turretAngle)), new Rotation2d())));
    Logger.recordOutput("Shooter/angleToTarget",
        targetOffset.getNorm() != 0 ? targetOffset.getAngle() : new Rotation2d());
    Logger.recordOutput("Shooter/robotRotation", robotPose.get().getRotation());

    return setpoint;
  }

  public TurretSetpoint getSetpointLocal() { // TODO not complete, nor advised!
    if (setpoint != null)
      return setpoint;

    // TODO change implementation in vision to return transform

    Transform3d tagVector = null;
    int tagId = -1;
    for (int id : hubTags.keySet()) {
      Optional<Translation3d> tagVectorOptional = turretCamera.getTrackingVector(id);
      if (tagVectorOptional.isPresent()) {
        tagVector = new Transform3d(tagVectorOptional.get(), new Rotation3d());
        tagId = id;
        break;
      }
    }

    Logger.recordOutput("Shooter/tagVector", tagVector);
    if (tagVector == null) {
      setpoint = lastSetpoint;
      return setpoint;
    }

    Translation2d centerToTag = new Pose3d().plus(turretCamera.getCameraTransform()).plus(tagVector)
        .getTranslation().toTranslation2d();

    Translation2d centerToHub = centerToTag.plus(hubTags.get(tagId).unaryMinus().rotateBy(new Rotation2d(
        -(robotRotation.get().getRadians() + turretAngle.getAsDouble() + turretZeroOffsetRobotFrame))));

    double delta = centerToHub.getAngle().getRadians();
    double angleSetpoint = turretAngle.getAsDouble() + delta;

    TurretSetpoint output = new TurretSetpoint(angleSetpoint, (angleSetpoint - lastSetpoint.turretAngle()) / 0.02,
        /* deflectorAngle */0, /* flywheelSpeed */0);

    lastSetpoint = setpoint;
    setpoint = output;

    Logger.recordOutput("Shooter/turretTargeting", robotPose.get()
        .plus(new Transform2d(new Translation2d(1, new Rotation2d(angleSetpoint)), new Rotation2d())));
    Logger.recordOutput("Shooter/tagPosRobotSpace",
        robotPose.get().plus(new Transform2d(new Translation2d(), new Rotation2d(turretAngle.getAsDouble())))
            .plus(new Transform2d(centerToTag, new Rotation2d())));
    Logger.recordOutput("Shooter/hubPosRobotSpace",
        robotPose.get().plus(new Transform2d(new Translation2d(), new Rotation2d(turretAngle.getAsDouble())))
            .plus(new Transform2d(centerToHub, new Rotation2d())));
    Logger.recordOutput("Shooter/centerToTag", centerToTag);
    Logger.recordOutput("Shooter/tagVector", tagVector);
    Logger.recordOutput("Shooter/delta", delta);
    Logger.recordOutput("Shooter/angleSetpoint", angleSetpoint);
    return setpoint;
  }
  
  public static Pose2d getNearestShootingPoint(Pose2d robotPose) {
    double x = robotPose.getX();
    double blueBoundaryX = FieldConstants.hubPositionBlue.getX();
    double redBoundaryX = FieldConstants.hubPositionRed.getX();

    boolean inBlueAllianceZone = x <= blueBoundaryX;
    boolean inRedAllianceZone = x >= redBoundaryX;

    boolean inOurAllianceZone = AllianceManager.chooseFromAlliance(inBlueAllianceZone, inRedAllianceZone);
    boolean inEnemyAllianceZone = AllianceManager.chooseFromAlliance(inRedAllianceZone, inBlueAllianceZone);
    if (inOurAllianceZone) {
      return AllianceManager.chooseFromAlliance(FieldConstants.hubPositionBlue, FieldConstants.hubPositionRed);
    }
    if (inEnemyAllianceZone) {
      return DistanceManager.getNearestPosition(robotPose, FieldConstants.neutralShootPoints);
    }
    Pose2d[] points = AllianceManager.chooseFromAlliance(FieldConstants.blueShootPoints,
        FieldConstants.redShootPoints);
    return DistanceManager.getNearestPosition(robotPose, points);

  }
}
