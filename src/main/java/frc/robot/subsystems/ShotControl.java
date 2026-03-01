package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.helpers.AllianceManager;
import frc.robot.util.helpers.DistanceManager;

public class ShotControl {

  private Supplier<Pose2d> robotPose;
  private Supplier<ChassisSpeeds> robotRelativeVelocity;

  private ShotType lastShotType;
  private ShotType shotType;

  private static ShotControl instance;

  public static record TurretSetpoint(double turretAngleRad, double turretOmegaRadPerSec, double hoodAngleDeg,
      double shooterVelocityRPM) {

  }

  public enum ShotType {
    SCORING, FERRYING
  }

  private TurretSetpoint setpoint;
  private TurretSetpoint lastSetpoint;

  private boolean isShooting = false;

  private static final InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToShooterVelocity = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap shooterVelocityToRPM45degHood = new InterpolatingDoubleTreeMap();

  public static final double expectedPosePhaseDelay = 0;

  public static final double shooterAngleFerry = Math.PI / 4;

  static {
    // distance (m) -> hood angle (deg)
    distanceToHoodAngle.put(1.5, 58.0);
    distanceToHoodAngle.put(2.0, 52.0);
    distanceToHoodAngle.put(2.5, 47.0);
    distanceToHoodAngle.put(3.0, 43.0);
    distanceToHoodAngle.put(3.5, 40.0);
    distanceToHoodAngle.put(4.0, 37.0);
    distanceToHoodAngle.put(4.5, 35.0);
    distanceToHoodAngle.put(5.0, 33.0);
    distanceToHoodAngle.put(5.5, 31.0);
    // custom format
        // TODO: THESE ARE DUMMY VALUES!!!!!!!!
        // spotless format
    // distance (m) -> shooter surface RPM
    distanceToShooterVelocity.put(1.5, 3200.0);
    distanceToShooterVelocity.put(2.0, 3400.0);
    distanceToShooterVelocity.put(2.5, 3600.0);
    distanceToShooterVelocity.put(3.0, 3800.0);
    distanceToShooterVelocity.put(3.5, 4000.0);
    distanceToShooterVelocity.put(4.0, 4200.0);
    distanceToShooterVelocity.put(4.5, 4400.0);
    distanceToShooterVelocity.put(5.0, 4600.0);
    distanceToShooterVelocity.put(5.5, 4800.0);

    // DUMMY VALUES
    shooterVelocityToRPM45degHood.put(1.0, 1000.0);
    shooterVelocityToRPM45degHood.put(2.0, 2000.0);
    shooterVelocityToRPM45degHood.put(3.0, 3000.0);
    shooterVelocityToRPM45degHood.put(4.0, 4000.0);
    shooterVelocityToRPM45degHood.put(5.0, 5000.0);
  }

  public ShotControl(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotRelativeVelocity, ShotType shotType) {
    this.robotPose = robotPose;
    this.robotRelativeVelocity = robotRelativeVelocity;
    setShotType(shotType);
    this.lastShotType = shotType;

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
    ShotControl.instance = this;

    Logger.recordOutput("Analysis/record", false);

  }

  public static ShotControl getInstance() {
    return instance;
  }

  public static void iterate() {
    instance.lastSetpoint = instance.setpoint;
    instance.setpoint = null;

    instance.lastShotType = instance.shotType;
  }

  public void setShotType(ShotType shotType) {
    Logger.recordOutput("Shot/Type", shotType);
    Logger.recordOutput("Shot/LastType", shotType);

    this.lastShotType = this.shotType;
    this.shotType = shotType;
  }

  public TurretSetpoint getSetpoint() {
    // sync logic
    if (setpoint != null) {
            Logger.recordOutput("Shot/setpoint", setpoint);

      return setpoint;
    }

    TurretSetpoint output = switch (shotType) {
      case SCORING -> getScoringSetpoint();
      case FERRYING -> getFerryingSetpoint();
    };

    if (shotType != lastShotType) { // prevent high velocities when shot type changes
      output = new TurretSetpoint(output.turretAngleRad, 0, output.hoodAngleDeg, output.shooterVelocityRPM);
    }

    lastSetpoint = setpoint;
    setpoint = output;
    
    Logger.recordOutput("Shot/setpoint", setpoint);
    return output;
  }

  private TurretSetpoint getScoringSetpoint() {
    // current robot velocity and turret velocity
    ChassisSpeeds velocity = robotRelativeVelocity.get();

    Translation2d fieldRelativeVelocity = new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond)
        .rotateBy(robotPose.get().getRotation());

    Pose2d turretPose = robotPose.get().exp(robotRelativeVelocity.get().toTwist2d(expectedPosePhaseDelay))
        .plus(TurretConstants.turretTransform2d); // fieldcentric

    // calculate distance to target
    Translation2d targetOffset = getNearestShootingPoint(robotPose.get()).getTranslation()
        .minus(turretPose.getTranslation()); // fieldcentric

    // use lookup tables to get hood angle and shooter speed
    double shooterVelocity = distanceToShooterVelocity.get(targetOffset.getNorm()); // without compensation
    double hoodAngle = distanceToHoodAngle.get(targetOffset.getNorm());

    Translation2d turretVelocity = turretPose.getTranslation().minus(robotPose.get().getTranslation()) // fieldcentric
        .rotateBy(Rotation2d.kCCW_Pi_2).times(velocity.omegaRadiansPerSecond).plus(fieldRelativeVelocity);

    // calculate turret angle setpoint
    Translation2d planarProjectileVelocity = new Translation2d(
        shooterVelocity * Math.cos(Math.toRadians(hoodAngle)), targetOffset.getAngle()); // fieldcentric

    planarProjectileVelocity = planarProjectileVelocity.minus(turretVelocity); // fieldcentric, compensated for
    // moving

    double turretAngle = planarProjectileVelocity.getNorm() != 0 // robotcentric
        ? planarProjectileVelocity.getAngle()
            .minus(robotPose.get().getRotation()
                .plus(new Rotation2d(TurretConstants.turretZeroOffsetRobotFrame)))
            .getRadians()
        : 0;

    double desiredTurretVelocity = lastSetpoint != null ? (turretAngle - lastSetpoint.turretAngleRad()) / 0.02 : 0;

    shooterVelocity = Math.hypot(planarProjectileVelocity.getNorm(),
        shooterVelocity * Math.sin(Math.toRadians(hoodAngle)));

    TurretSetpoint output = new TurretSetpoint(turretAngle, desiredTurretVelocity,
        Units.radiansToDegrees(Math.acos(planarProjectileVelocity.getNorm() / shooterVelocity)),
        shooterVelocity);

    return output;
  }

  private TurretSetpoint getFerryingSetpoint() {
    Pose2d turretPose = robotPose.get().exp(robotRelativeVelocity.get().toTwist2d(expectedPosePhaseDelay))
        .plus(TurretConstants.turretTransform2d); // fieldcentric

    // calculate distance to target
    Translation2d targetOffset = getNearestShootingPoint(turretPose).getTranslation()
        .minus(turretPose.getTranslation()); // fieldcentric

    // use the math to calculate velocity. Hood angle at 45 degrees
    double shooterVelocity = Math
        .sqrt((FieldConstants.gravityEarth * targetOffset.getNorm()) / Math.sin(2 * shooterAngleFerry));
    Rotation2d hoodAngle = new Rotation2d(shooterAngleFerry);
    double desiredTurretVelocity = lastSetpoint != null
        ? (hoodAngle.getRadians() - lastSetpoint.turretAngleRad()) / 0.02
        : 0;

    TurretSetpoint output = new TurretSetpoint(
        targetOffset.getAngle().minus(robotPose.get().getRotation()).getRadians(), desiredTurretVelocity,
        hoodAngle.getDegrees(), shooterVelocity);

    return output;

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

  public boolean isShooting() {
    return isShooting;
  }

  public void setShooting(boolean shooting) {
    isShooting = shooting;
    Logger.recordOutput("Analysis/record", shooting);
  }
}
