package frc.robot.subsystems;

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
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.helpers.AllianceManager;
import frc.robot.util.helpers.DistanceManager;

public class ShotControl {

  private Supplier<Pose2d> robotPose;
  private Supplier<ChassisSpeeds> robotRelativeVelocity;

  private ShotType lastShotType;
  private boolean manualShots = false;

  private static ShotControl instance;

  public static record ShotSetpoint(double turretAngleRad, double turretOmegaRadPerSec, double hoodAngleDeg,
      double shooterVelocityRPM) {

  }

  public enum ShotType {
    SCORING, FERRYING, STEALING, MANUAL
  }

  public static final ShotSetpoint towerManualSetpoint = new ShotSetpoint(Math.PI / 2, 0, 15.0, 3000.0);
  public static final ShotSetpoint leftTrenchManualSetpoint = new ShotSetpoint(Units.degreesToRadians(110), 0, 21.0,
      3220.0);
  public static final ShotSetpoint rightTrenchManualSetpoint = new ShotSetpoint(-Units.degreesToRadians(110), 0, 23.0,
      3315.0);

  private Zone currentZone;

  private ShotSetpoint setpoint;
  private ShotSetpoint lastSetpoint;

  private ShotSetpoint manualSetpoint = new ShotSetpoint(0, 0, 15.0, 0);

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
    // distanceToHoodAngleAZ.put(1.872, 15.0);
    // distanceToHoodAngleAZ.put(2.228, 16.0);
    // distanceToHoodAngleAZ.put(2.442, 20.0);
    // distanceToHoodAngleAZ.put(2.905, 21.0);
    // distanceToHoodAngleAZ.put(3.384, 26.2);
    // distanceToHoodAngleAZ.put(4.000, 26.4);
    // distanceToHoodAngleAZ.put(4.604, 26.0);
    // distanceToHoodAngleAZ.put(5.433, 27.0);
    distanceToHoodAngleAZ.put(1.705, 14.8); // TOF = 0.75 s
    distanceToHoodAngleAZ.put(2.078, 15.0); // 2.33045
    distanceToHoodAngleAZ.put(2.553, 15.25);
    distanceToHoodAngleAZ.put(3.162, 15.25); // TOF = 0.8 s
    distanceToHoodAngleAZ.put(3.645, 20.4); // TOF = 0.925 s
    distanceToHoodAngleAZ.put(4.046, 23.7); // TOF = 1.25 s
    distanceToHoodAngleAZ.put(4.578, 25.0);
    distanceToHoodAngleAZ.put(5.225, 27.504);

    // distance (m) -> shooter surface RPM in Alliance Zone
    // distanceToShooterVelocityAZ.put(1.872, 2700.0);
    // distanceToShooterVelocityAZ.put(2.228, 2800.0);
    // distanceToShooterVelocityAZ.put(2.442, 2800.0);
    // distanceToShooterVelocityAZ.put(2.905, 2900.0);
    // distanceToShooterVelocityAZ.put(3.384, 3120.0);
    // distanceToShooterVelocityAZ.put(4.000, 3230.0);
    // distanceToShooterVelocityAZ.put(4.604, 3450.0);
    // distanceToShooterVelocityAZ.put(5.433, 3750.0);
    distanceToShooterVelocityAZ.put(1.705, 2500.0);
    distanceToShooterVelocityAZ.put(2.078, 2620.0); // 2.33045
    distanceToShooterVelocityAZ.put(2.553, 2850.0);
    distanceToShooterVelocityAZ.put(3.162, 3135.0);
    distanceToShooterVelocityAZ.put(3.645, 3200.0);
    distanceToShooterVelocityAZ.put(4.046, 3325.0);
    distanceToShooterVelocityAZ.put(4.578, 3465.0);
    distanceToShooterVelocityAZ.put(5.225, 3690.0);

    // distance (m) -> hood angle (deg) in Neutral Zone
    distanceToHoodAngleNZ.put(4.438, 34.5);
    distanceToHoodAngleNZ.put(5.027, 34.5);
    distanceToHoodAngleNZ.put(6.243, 34.5);
    distanceToHoodAngleNZ.put(7.253, 30.0);
    distanceToHoodAngleNZ.put(8.289, 27.0);

    // distance (m) -> shooter surface RPM in Neutral Zone
    distanceToShooterVelocityNZ.put(4.438, 3100.0);
    distanceToShooterVelocityNZ.put(5.027, 3150.0);
    distanceToShooterVelocityNZ.put(6.243, 3300.0);
    distanceToShooterVelocityNZ.put(7.253, 4000.0);
    distanceToShooterVelocityNZ.put(8.289, 4350.0);

    Logger.recordOutput("FerryingTargets", new Pose2d[]{FieldConstants.redShootNorth, FieldConstants.redShootSouth,
        FieldConstants.blueShootNorth, FieldConstants.blueShootSouth});

    // DUMMY VALUES
    shooterVelocityToRPM45degHood.put(1.0, 1000.0);
    shooterVelocityToRPM45degHood.put(2.0, 2000.0);
    shooterVelocityToRPM45degHood.put(3.0, 3000.0);
    shooterVelocityToRPM45degHood.put(4.0, 4000.0);
    shooterVelocityToRPM45degHood.put(5.0, 5000.0);
  }

  public ShotControl(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotRelativeVelocity) {
    this.robotPose = robotPose;
    this.currentZone = AllianceManager.chooseFromAlliance(Zone.BLUE_ALLIANCE, Zone.RED_ALLIANCE);

    this.robotRelativeVelocity = robotRelativeVelocity;
    this.lastShotType = ShotType.SCORING;

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
  }

  public void setManual(boolean manual) {
    this.manualShots = manual;
  }

  public void setManualSetpoint(ShotSetpoint setpoint) {
    manualSetpoint = setpoint;
  }

  public Zone getZone() {
    return currentZone;
  }

  public ShotSetpoint getSetpoint() {
    // sync logic
    if (setpoint != null) {
      Logger.recordOutput("Shot/setpoint", setpoint);
      return setpoint;
    }
    if (manualShots) {
      lastSetpoint = setpoint;
      setpoint = manualSetpoint;

      return manualSetpoint;
    }
    Pose2d turretPose = robotPose.get().exp(robotRelativeVelocity.get().toTwist2d(expectedPosePhaseDelay))
        .plus(TurretConstants.turretTransform2d);

    ShotType shotType = getShotMode(turretPose);
    lastShotType = shotType;
    Logger.recordOutput("Shot/shotType", shotType);

    Pose2d target = DistanceManager.getNearestPosition(turretPose, getShotTargets(shotType));
    Logger.recordOutput("Shot/target", target);
    Logger.recordOutput("DistanceToFerry",
        RobotOdometry.instance.getPose("Main").getTranslation().getDistance(target.getTranslation()));

    ShotSetpoint output = calculateShot(target, robotPose.get(), robotRelativeVelocity.get(),
        TurretConstants.turretTransform2d, shotType);
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
    Logger.recordOutput("Shot/targetOffset", targetOffset);

    double targetDistance = targetOffset.getNorm();
    double shooterVelocity = 0;
    double hoodAngle = 0;

    // use lookup tables to get hood angle and shooter speed
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

    // calculate turret angle setpoint
    Translation2d planarProjectileVelocity = new Translation2d(
        shooterVelocity * Math.cos(Math.toRadians(hoodAngle)), targetOffset.getAngle()); // fieldcentric

    planarProjectileVelocity = planarProjectileVelocity.minus(turretVelocity); // fieldcentric, compensated for
    // moving

    double turretAngle = /* planarProjectileVelocity */targetOffset.getNorm() != 0 // robotcentric
        ? /* planarProjectileVelocity */targetOffset.getAngle()
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

    Logger.recordOutput("currentzone", currentZone);
    Logger.recordOutput("switchzone", switchZone);

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

  private Pose2d[] getShotTargets(ShotType shotType) {
    return switch (shotType) {
      case SCORING, MANUAL -> AllianceManager.chooseFromAlliance(new Pose2d[]{FieldConstants.hubPositionBlue},
          new Pose2d[]{FieldConstants.hubPositionRed});
      case FERRYING -> AllianceManager.chooseFromAlliance(FieldConstants.blueShootPoints,
          FieldConstants.redShootPoints);
      case STEALING -> FieldConstants.neutralShootPoints;
    };
  }

  public boolean isShooting() {
    return isShooting;
  }

  public void setShooting(boolean shooting) {
    isShooting = shooting;
    Logger.recordOutput("Analysis/record", shooting);
  }
}
