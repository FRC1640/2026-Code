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
  private Supplier<Pose2d> targetPose;

  private ShotType lastShotType;
  private ShotType shotType;

  private static ShotControl instance;

  public static record TurretSetpoint(double turretAngleRad, double turretOmegaRadPerSec, double hoodAngleDeg,
      double shooterVelocityRPM) {
  }

  public enum ShotType {
    SCORING, FERRYING, MANUAL
  }

  private TurretSetpoint setpoint;
  private TurretSetpoint lastSetpoint;

  private boolean isShooting = false;

  private static final InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToShooterVelocity = new InterpolatingDoubleTreeMap();

  public static final double expectedPosePhaseDelay = 0;

  static {
    // distance (m) -> hood angle (deg)
    // distanceToHoodAngle.put(1.5, 58.0);
    // distanceToHoodAngle.put(2.0, 52.0);
    // distanceToHoodAngle.put(2.5, 47.0);
    // distanceToHoodAngle.put(3.0, 43.0);
    // distanceToHoodAngle.put(3.5, 40.0);
    // distanceToHoodAngle.put(4.0, 37.0);
    // distanceToHoodAngle.put(4.5, 35.0);
    // distanceToHoodAngle.put(5.0, 33.0);
    // distanceToHoodAngle.put(5.5, 31.0);
    distanceToHoodAngle.put(1.872, 14.0);
    distanceToHoodAngle.put(2.228, 16.0);
    distanceToHoodAngle.put(2.442, 20.0);
    distanceToHoodAngle.put(2.905, 21.0);
    distanceToHoodAngle.put(3.384, 26.2);
    // custom format
                                                              // TODO: THESE ARE DUMMY VALUES!!!!!!!!
                                                              // spotless format
    // distance (m) -> shooter surface RPM
    // distanceToShooterVelocity.put(1.5, 3200.0);
    // distanceToShooterVelocity.put(2.0, 3400.0);
    // distanceToShooterVelocity.put(2.5, 3600.0);
    // distanceToShooterVelocity.put(3.0, 3800.0);
    // distanceToShooterVelocity.put(3.5, 4000.0);
    // distanceToShooterVelocity.put(4.0, 4200.0);
    // distanceToShooterVelocity.put(4.5, 4400.0);
    // distanceToShooterVelocity.put(5.0, 4600.0);
    // distanceToShooterVelocity.put(5.5, 4800.0);
    distanceToShooterVelocity.put(1.872, 2800.0);
    distanceToShooterVelocity.put(2.228, 2800.0);
    distanceToShooterVelocity.put(2.442, 2800.0);
    distanceToShooterVelocity.put(2.905, 2900.0);
    distanceToShooterVelocity.put(3.384, 3120.0);
  }

  public ShotControl(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotRelativeVelocity,
      Supplier<Pose2d> targetPose, ShotType shotType) {
    this.robotPose = robotPose;
    this.robotRelativeVelocity = robotRelativeVelocity;
    this.targetPose = targetPose;
    this.shotType = shotType;
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
    this.shotType = shotType;
  }

  public void setSetpoint(TurretSetpoint setpoint) {
    if (shotType == ShotType.MANUAL)
      this.setpoint = setpoint;
  }

  public TurretSetpoint getSetpoint() {
    // sync logic
    if (setpoint != null) {
      return setpoint;
    }

    TurretSetpoint output = switch (shotType) {
      case SCORING -> getScoringSetpoint();
      case FERRYING -> getFerryingSetpoint();
      case MANUAL -> getManualSetpoint();
    };

    // TODO: is this really necessary? We can account for this with PID and FF and
    // besides, it just delays the high velocities for 20ms
    if (shotType != lastShotType) { // prevent high velocities when shot type changes
      output = new TurretSetpoint(output.turretAngleRad, 0, output.hoodAngleDeg, output.shooterVelocityRPM);
    }

    lastSetpoint = setpoint;
    setpoint = output;

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
    Translation2d targetOffset = targetPose.get().getTranslation().minus(turretPose.getTranslation()); // fieldcentric

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
    return getScoringSetpoint();
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

  private TurretSetpoint getManualSetpoint() {
    return setpoint;
  }

  public boolean isShooting() {
    return isShooting;
  }

  public void setShooting(boolean shooting) {
    isShooting = shooting;
    Logger.recordOutput("Analysis/record", shooting);
  }
}
