package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Zone;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.helpers.AllianceManager;
import frc.robot.util.helpers.DistanceManager;

public class ShotControl {

  private Supplier<Pose2d> robotPose;
  private Supplier<ChassisSpeeds> robotRelativeVelocity;

  private ShotType lastShotType;
  private ShotType shotType;

  private static ShotControl instance;

  public static record ShotSetpoint(double turretAngleRad, double turretOmegaRadPerSec, double hoodAngleDeg,
      double shooterVelocityRPM) {

  }

  public enum ShotType {
    SCORING, FERRYING, STEALING, MANUAL
  }

  private static final Map<ShotType, Pose2d[]> shotTargets = new HashMap<>();

  private Zone currentZone;

  private ShotSetpoint setpoint;
  private ShotSetpoint lastSetpoint;

  private boolean isShooting = false;

  private static final InterpolatingDoubleTreeMap distanceToHoodAngleAZ = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToShooterVelocityAZ = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap shooterVelocityToRPM45degHood = new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap distanceToHoodAngleNZ = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToShooterVelocityNZ = new InterpolatingDoubleTreeMap();

  public static final double expectedPosePhaseDelay = 0;

  public static final double shooterAngleFerry = Math.PI / 4;

  static {
    // distance (m) -> hood angle (deg) in Alliance Zone
    distanceToHoodAngleAZ.put(1.872, 15.0);
    distanceToHoodAngleAZ.put(2.228, 16.0);
    distanceToHoodAngleAZ.put(2.442, 20.0);
    distanceToHoodAngleAZ.put(2.905, 21.0);
    distanceToHoodAngleAZ.put(3.384, 26.2);
    distanceToHoodAngleAZ.put(4.000, 26.4);
    distanceToHoodAngleAZ.put(4.604, 26.0);
    distanceToHoodAngleAZ.put(5.433, 27.0);

    // distance (m) -> shooter surface RPM in Alliance Zone
    distanceToShooterVelocityAZ.put(1.872, 2700.0);
    distanceToShooterVelocityAZ.put(2.228, 2800.0);
    distanceToShooterVelocityAZ.put(2.442, 2800.0);
    distanceToShooterVelocityAZ.put(2.905, 2900.0);
    distanceToShooterVelocityAZ.put(3.384, 3120.0);
    distanceToShooterVelocityAZ.put(4.000, 3230.0);
    distanceToShooterVelocityAZ.put(4.604, 3450.0);
    distanceToShooterVelocityAZ.put(5.433, 3750.0);

    // distance (m) -> hood angle (deg) in Neutral Zone
    distanceToHoodAngleNZ.put(1.0, 25.0);
    distanceToHoodAngleNZ.put(2.0, 35.0);
    distanceToHoodAngleNZ.put(3.0, 45.0);

    // distance (m) -> shooter surface RPM in Neutral Zone
    distanceToShooterVelocityNZ.put(1.0, 2000.0);
    distanceToShooterVelocityNZ.put(2.0, 3000.0);
    distanceToShooterVelocityNZ.put(3.0, 5000.0);

    shotTargets.put(ShotType.SCORING, AllianceManager.chooseFromAlliance(
        new Pose2d[]{FieldConstants.hubPositionBlue}, new Pose2d[]{FieldConstants.hubPositionRed}));
    shotTargets.put(ShotType.FERRYING,
        AllianceManager.chooseFromAlliance(FieldConstants.blueShootPoints, FieldConstants.redShootPoints));
    shotTargets.put(ShotType.STEALING, FieldConstants.neutralShootPoints);

    // DUMMY VALUES
    shooterVelocityToRPM45degHood.put(1.0, 1000.0);
    shooterVelocityToRPM45degHood.put(2.0, 2000.0);
    shooterVelocityToRPM45degHood.put(3.0, 3000.0);
    shooterVelocityToRPM45degHood.put(4.0, 4000.0);
    shooterVelocityToRPM45degHood.put(5.0, 5000.0);
  }

  public ShotControl(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotRelativeVelocity, ShotType shotType) {
    this.robotPose = robotPose;

    double x = this.robotPose.get().getX();
    double blueBoundaryX = FieldConstants.hubPositionBlue.getX();
    double redBoundaryX = FieldConstants.hubPositionRed.getX();

    this.currentZone = x <= blueBoundaryX
        ? Zone.BLUE_ALLIANCE
        : x >= redBoundaryX ? Zone.RED_ALLIANCE : Zone.NEUTRAL;

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
    setpoint = new ShotSetpoint(0, 0, 0, 0);
    lastSetpoint = new ShotSetpoint(0, 0, 0, 0);
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

  public void setSetpoint(ShotSetpoint setpoint) {
    if (shotType == ShotType.MANUAL)
      this.setpoint = setpoint;
  }

  public ShotSetpoint getSetpoint() {
    // sync logic
    if (setpoint != null) {
      Logger.recordOutput("Shot/setpoint", setpoint);
      return setpoint;
    }
    Pose2d turretPose = robotPose.get().exp(robotRelativeVelocity.get().toTwist2d(expectedPosePhaseDelay))
        .plus(TurretConstants.turretTransform2d);
    Pose2d target = DistanceManager.getNearestPosition(turretPose, shotTargets.get(getShotMode(turretPose)));
    ShotSetpoint output = calculateShot(target, robotPose.get(), robotRelativeVelocity.get(),
        TurretConstants.turretTransform2d, getShotMode(turretPose));
    // TODO: is this really necessary? We can account for this with PID and FF and
    // besides, it just delays the high velocities for 20ms
    if (shotType != lastShotType) { // prevent high velocities when shot type changes
      output = new ShotSetpoint(output.turretAngleRad, 0, output.hoodAngleDeg, output.shooterVelocityRPM);
    }

    lastSetpoint = setpoint;
    setpoint = output;

    Logger.recordOutput("Shot/setpoint", setpoint);
    return output;
  }
  private ShotSetpoint getFerryingSetpoint() {
    Pose2d turretPose = robotPose.get().exp(robotRelativeVelocity.get().toTwist2d(expectedPosePhaseDelay))
        .plus(TurretConstants.turretTransform2d); // fieldcentric

    Logger.recordOutput("Shot/target", getShotMode(turretPose));

    // calculate distance to target
    Pose2d target = DistanceManager.getNearestPosition(turretPose, shotTargets.get(getShotMode(turretPose)));

    Translation2d targetOffset = target.getTranslation().minus(turretPose.getTranslation()); // fieldcentric

    // use the math to calculate velocity. Hood angle at 45 degrees
    double shooterVelocity = Math
        .sqrt((FieldConstants.gravityEarth * targetOffset.getNorm()) / Math.sin(2 * shooterAngleFerry));
    Rotation2d hoodAngle = new Rotation2d(shooterAngleFerry);

    double turretAngle = targetOffset.getAngle()
        .minus(robotPose.get().getRotation()
            .plus(new Rotation2d(TurretConstants.turretTransform2d.getRotation().getRadians())))
        .getRadians();

    double desiredTurretVelocity = lastSetpoint != null
        ? (hoodAngle.getRadians() - lastSetpoint.turretAngleRad()) / 0.02
        : 0;

    ShotSetpoint output = new ShotSetpoint(turretAngle, desiredTurretVelocity, hoodAngle.getDegrees(),
        shooterVelocity);

    return output;
  }

  private ShotSetpoint calculateShot(Pose2d targetPose, Pose2d robotPose, ChassisSpeeds robotRelativeVelocity,
      Transform2d turretTransformRobotFrame, ShotType shotType) {
    Pose2d turretPose = robotPose.exp(robotRelativeVelocity.toTwist2d(expectedPosePhaseDelay))
        .plus(turretTransformRobotFrame); // field-space turret pose

    Translation2d fieldRelativeVelocity = new Translation2d(robotRelativeVelocity.vxMetersPerSecond,
        robotRelativeVelocity.vyMetersPerSecond).rotateBy(robotPose.getRotation());

    // field-space turret velocity
    Translation2d turretVelocity = turretPose.getTranslation().minus(robotPose.getTranslation())
        .rotateBy(Rotation2d.kCCW_Pi_2).times(robotRelativeVelocity.omegaRadiansPerSecond)
        .plus(fieldRelativeVelocity);

    // field-space vector from turret to target
    Translation2d targetOffset = targetPose.getTranslation().minus(turretPose.getTranslation());

    double targetDistance = targetOffset.getNorm();
    double shooterVelocity = 0;
    double hoodAngle = 0;

    switch (shotType) {
      case SCORING -> {
        shooterVelocity = distanceToShooterVelocityAZ.get(targetDistance);
        hoodAngle = distanceToHoodAngleAZ.get(targetDistance);
      }
      case FERRYING -> {
        shooterVelocity = distanceToShooterVelocityNZ.get(targetDistance);
        hoodAngle = distanceToHoodAngleNZ.get(targetDistance);
      }
      case STEALING -> {
        shooterVelocity = distanceToShooterVelocityNZ.get(targetDistance);
        hoodAngle = distanceToShooterVelocityNZ.get(targetDistance);
      }
      default -> {
        shooterVelocity = 0;
        hoodAngle = 0;
      }
    }
    // use lookup tables to get hood angle and shooter speed

    // calculate turret angle setpoint
    Translation2d planarProjectileVelocity = new Translation2d(
        shooterVelocity * Math.cos(Math.toRadians(hoodAngle)), targetOffset.getAngle()); // fieldcentric

    planarProjectileVelocity = planarProjectileVelocity.minus(turretVelocity); // fieldcentric, compensated for
    // moving

    double turretAngle = planarProjectileVelocity.getNorm() != 0 // robotcentric
        ? planarProjectileVelocity.getAngle()
            .minus(robotPose.getRotation()
                .plus(new Rotation2d(turretTransformRobotFrame.getRotation().getRadians())))
            .getRadians()
        : 0;

    double desiredTurretVelocity = lastSetpoint != null ? (turretAngle - lastSetpoint.turretAngleRad()) / 0.02 : 0;

    shooterVelocity = Math.hypot(planarProjectileVelocity.getNorm(),
        shooterVelocity * Math.sin(Math.toRadians(hoodAngle)));
    hoodAngle = Units.radiansToDegrees(Math.acos(planarProjectileVelocity.getNorm() / shooterVelocity));

    ShotSetpoint output = new ShotSetpoint(turretAngle, desiredTurretVelocity, hoodAngle, shooterVelocity);

    return output;
  }

  private ShotType getShotMode(Pose2d turretPose) {
    double x = turretPose.getX();
    double blueBoundaryX = FieldConstants.hubPositionBlue.getX();
    double redBoundaryX = FieldConstants.hubPositionRed.getX();

    Zone switchZone = currentZone;

    double zsh = RobotConstants.zoneSwitchingHysteresis;

    switchZone = x <= blueBoundaryX - zsh ? Zone.BLUE_ALLIANCE : switchZone;
    switchZone = x >= redBoundaryX + zsh ? Zone.RED_ALLIANCE : switchZone;
    switchZone = x > blueBoundaryX + zsh && x < redBoundaryX - zsh ? Zone.NEUTRAL : switchZone;
    currentZone = switchZone;

    boolean inOurAllianceZone = AllianceManager.chooseFromAlliance(currentZone == Zone.BLUE_ALLIANCE,
        currentZone == Zone.RED_ALLIANCE);
    boolean inEnemyAllianceZone = AllianceManager.chooseFromAlliance(currentZone == Zone.RED_ALLIANCE,
        currentZone == Zone.BLUE_ALLIANCE);
    Logger.recordOutput("inOurAllianceZone", inOurAllianceZone);
    Logger.recordOutput("inEnemyAllianceZone", inEnemyAllianceZone);
    Logger.recordOutput("inNeutralZone", !inOurAllianceZone && !inEnemyAllianceZone);
    if (inOurAllianceZone) {
      return ShotType.SCORING;
    }
    if (inEnemyAllianceZone) {
      return ShotType.STEALING;
    }
    return ShotType.FERRYING;
  }

  private ShotSetpoint getManualSetpoint() {
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
