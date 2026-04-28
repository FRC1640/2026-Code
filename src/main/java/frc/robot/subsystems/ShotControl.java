package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Zone;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.helpers.AllianceManager;
import frc.robot.util.helpers.DistanceManager;
import frc.robot.util.math.LookupTableSlurper;
import frc.robot.util.math.ShotInterpolator;
import frc.robot.util.math.LookupTableSlurper.LookupTableType;

public class ShotControl {

  private Supplier<Pose2d> robotPose;
  private Supplier<ChassisSpeeds> robotRelativeVelocity;

  private ShotType lastShotType;
  private boolean manualShots = false;

  private boolean shotFlag;

  private static ShotControl instance;

  public static record ShotSetpoint(double turretAngleRad, double turretOmegaRadPerSec, double hoodAngleDeg,
      double shooterVelocityRPM, double shooterAccelerationRotationsPerMinuteSquared) {

  }

  public enum ShotType {
    SCORING, FERRYING, STEALING, MANUAL
  }

  public static final ShotSetpoint towerManualSetpoint = new ShotSetpoint(Math.PI / 2, 0, 32.0, 2820.0, 0);
  public static final ShotSetpoint leftTrenchManualSetpoint = new ShotSetpoint(1.836, 0, 35.0, 2890.0, 0);
  public static final ShotSetpoint rightTrenchManualSetpoint = new ShotSetpoint(-1.899, 0, 36.0, 3030.0, 0);

  private Zone currentZone;

  private ShotSetpoint setpoint;
  private ShotSetpoint lastSetpoint;

  private ShotSetpoint manualSetpoint = new ShotSetpoint(0, 0, 15.0, 0, 0);

  private boolean isShooting = false;

  private Pair<Pose2d, ShotType> targetOverride; // field-centric target pose and shot type

  public static final ShotInterpolator AZInterpolator = LookupTableSlurper
      .slurpShotInterpolator(LookupTableType.PRIME26AZ);
  public static final ShotInterpolator NZInterpolator = LookupTableSlurper
      .slurpShotInterpolator(LookupTableType.PRIME26NZ);

  public static final double expectedPosePhaseDelay = 0;

  public static final double shooterAngleFerry = Math.PI / 4;
  private static final double displacementThreshold = 0.1;

  private double hubShotOffset = 0.15;
  private boolean useHubShotOffset = true;

  public ShotControl(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotRelativeVelocity) {
    this.robotPose = robotPose;
    this.currentZone = Zone.ALLIANCE_ZONE;

    this.shotFlag = false;

    this.robotRelativeVelocity = robotRelativeVelocity;
    this.lastShotType = ShotType.SCORING;

    setpoint = new ShotSetpoint(0, 0, 0, 0, 0);
    lastSetpoint = new ShotSetpoint(0, 0, 0, 0, 0);
    ShotControl.instance = this;

    Logger.recordOutput("Analysis/record", false);

  }

  public static ShotControl getInstance() {
    return instance;
  }

  public void iterate() {
    // update zone
    Pose2d turretPose = robotPose.get().plus(TurretConstants.turretTransform2d);

    double x = turretPose.getX();
    double blueBoundaryX = FieldConstants.hubPositionBlue.getX();
    double redBoundaryX = FieldConstants.hubPositionRed.getX();

    Zone switchZone = currentZone;

    double zsh = RobotConstants.zoneSwitchingHysteresis;

    switchZone = x <= blueBoundaryX - zsh
        ? AllianceManager.chooseFromAlliance(Zone.ALLIANCE_ZONE, Zone.ENEMY_ZONE)
        : switchZone;
    switchZone = x >= redBoundaryX + zsh
        ? AllianceManager.chooseFromAlliance(Zone.ENEMY_ZONE, Zone.ALLIANCE_ZONE)
        : switchZone;
    switchZone = x > blueBoundaryX + zsh && x < redBoundaryX - zsh ? Zone.NEUTRAL_ZONE : switchZone;
    currentZone = switchZone;
    Logger.recordOutput("Shot/currentZone", currentZone);

    Logger.recordOutput("Shot/hubShotOffset", hubShotOffset);
    Logger.recordOutput("Shot/usingHubShotOffset", useHubShotOffset);
    Logger.recordOutput("Shot/shotFlag", this.shotFlag);

    // update setpoint
    lastSetpoint = setpoint;
    setpoint = null;
  }

  public void setTargetOverride(Pose2d targetPose, ShotType shotType) {
    this.targetOverride = new Pair<>(targetPose, shotType);
  }

  public void clearTargetOverride() {
    this.targetOverride = null;
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

    ShotType shotType = getShotMode();
    lastShotType = shotType;
    Logger.recordOutput("Shot/shotType", shotType);

    Pose2d targetPose = DistanceManager.getNearestPosition(turretPose, getShotTargets(shotType));
    Logger.recordOutput("Shot/target", targetPose);

    Logger.recordOutput("DistanceToTarget", turretPose.getTranslation().getDistance(targetPose.getTranslation()));

    ShotSetpoint output = calculateShot(targetPose, robotPose.get(), robotRelativeVelocity.get(),
        TurretConstants.turretTransform2d, shotType);

    if (shotType != lastShotType) { // prevent high velocities when shot type changes
      output = new ShotSetpoint(output.turretAngleRad, 0, output.hoodAngleDeg, output.shooterVelocityRPM, 0);
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
    Logger.recordOutput("Shot/turretPose", turretPose);
    Logger.recordOutput("Shot/turretVelocity", turretVelocity);
    Logger.recordOutput("Shot/fieldRelativeRobotVelocity", fieldRelativeVelocity);
    Logger.recordOutput("Shot/origin", new Pose2d());

    // field-space vector from turret to target
    Translation2d targetOffset = targetPose.getTranslation().minus(turretPose.getTranslation());
    Logger.recordOutput("Shot/targetOffset", targetOffset);

    double targetDistance = targetOffset.getNorm();
    double shooterVelocity = 0;
    double hoodAngle = 0;
    double timeOfFlight = 0;

    switch (shotType) {
      case SCORING -> {
        if (useHubShotOffset) {
          targetOffset = targetOffset.plus(new Translation2d(hubShotOffset, targetOffset.getAngle()));
          targetDistance = targetOffset.getNorm();
        }
        timeOfFlight = AZInterpolator.getTimeOfFlight(targetDistance);
      }
      case FERRYING -> {
        timeOfFlight = NZInterpolator.getTimeOfFlight(targetDistance);
      }
      case STEALING -> {
        timeOfFlight = NZInterpolator.getTimeOfFlight(targetDistance);
      }
      default -> {
      }
    }
    Logger.recordOutput("Shot/distanceAdjustedTarget",
        new Pose2d(turretPose.getTranslation().plus(targetOffset), targetOffset.getAngle()));
    Translation2d targetDisplacement = turretVelocity.times(timeOfFlight);
    targetOffset = targetOffset.plus(targetDisplacement.unaryMinus());
    double lastTimeOfFlight = timeOfFlight;
    for (int i = 0; i < 20; i++) {
      targetDistance = targetOffset.getNorm();
      // use lookup tables to get hood angle and shooter speed
      switch (shotType) {
        case SCORING -> {
          timeOfFlight = AZInterpolator.getTimeOfFlight(targetDistance);
        }
        case FERRYING -> {
          timeOfFlight = NZInterpolator.getTimeOfFlight(targetDistance);

        }
        case STEALING -> {
          timeOfFlight = NZInterpolator.getTimeOfFlight(targetDistance);
        }
        default -> {
        }
      }

      targetDisplacement = turretVelocity.times(timeOfFlight - lastTimeOfFlight);
      targetOffset = targetOffset.plus(targetDisplacement.unaryMinus());
      lastTimeOfFlight = timeOfFlight;

      if (targetDisplacement.getNorm() < displacementThreshold)
        break;
      if (i == 19)
        System.out.println("Loop forced to terminate in move and shoot iteration");
    }
    Logger.recordOutput("Shot/adjustedTarget",
        new Pose2d(targetOffset.plus(turretPose.getTranslation()), new Rotation2d()));

    switch (shotType) {
      case SCORING -> {
        shooterVelocity = AZInterpolator.getShooterVelocity(targetDistance);
        hoodAngle = AZInterpolator.getHoodAngle(targetDistance);
      }
      case FERRYING -> {
        shooterVelocity = NZInterpolator.getShooterVelocity(targetDistance);
        hoodAngle = NZInterpolator.getHoodAngle(targetDistance);
      }
      case STEALING -> {
        shooterVelocity = NZInterpolator.getShooterVelocity(targetDistance);
        hoodAngle = NZInterpolator.getHoodAngle(targetDistance);
      }
      default -> {
      }
    }
    double turretAngle = targetOffset.getNorm() != 0 // robotcentric
        ? targetOffset.getAngle()
            .minus(robotPose.getRotation()
                .plus(new Rotation2d(turretTransformRobotFrame.getRotation().getRadians())))
            .getRadians()
        : 0;

    double desiredTurretVelocity = lastSetpoint != null ? (turretAngle - lastSetpoint.turretAngleRad()) / 0.02 : 0;
    double desiredShooterAcceleration = lastSetpoint != null
        ? (shooterVelocity - lastSetpoint.shooterVelocityRPM()) / (0.02 / 60)
        : 0;

    ShotSetpoint output = new ShotSetpoint(turretAngle, desiredTurretVelocity, hoodAngle, shooterVelocity,
        desiredShooterAcceleration);

    return output;
  }

  private ShotType getShotMode() {
    boolean inOurAllianceZone = currentZone == Zone.ALLIANCE_ZONE;
    boolean inEnemyAllianceZone = currentZone == Zone.ENEMY_ZONE;
    Logger.recordOutput("inOurAllianceZone", inOurAllianceZone);
    Logger.recordOutput("inEnemyAllianceZone", inEnemyAllianceZone);
    Logger.recordOutput("inNeutralZone", !inOurAllianceZone && !inEnemyAllianceZone);

    if (this.targetOverride != null) {
      return this.targetOverride.getSecond();
    }

    if (inOurAllianceZone) {
      return ShotType.SCORING;
    }
    if (inEnemyAllianceZone) {
      return ShotType.STEALING;
    }
    return ShotType.FERRYING;
  }

  private Pose2d[] getShotTargets(ShotType shotType) {
    if (this.targetOverride != null) {
      return new Pose2d[]{this.targetOverride.getFirst()};
    }
    return switch (shotType) {
      case SCORING, MANUAL -> AllianceManager.chooseFromAlliance(new Pose2d[]{FieldConstants.hubPositionBlue},
          new Pose2d[]{FieldConstants.hubPositionRed});
      case FERRYING -> AllianceManager.chooseFromAlliance(FieldConstants.blueShootPoints,
          FieldConstants.redShootPoints);
      case STEALING -> AllianceManager.chooseFromAlliance(FieldConstants.blueStealPoints,
          FieldConstants.redStealPoints);
    };
  }

  public boolean isShooting() {
    return isShooting;
  }

  public void setShooting(boolean shooting) {
    isShooting = shooting;
    Logger.recordOutput("Analysis/record", shooting);
  }

  public void incrementHubShotOffset(double deltaMeters) {
    hubShotOffset += deltaMeters;
  }

  public void setOffsetHubShot(boolean useOffset) {
    this.useHubShotOffset = useOffset;
  }

  public void toggleOffsetHubShot() {
    setOffsetHubShot(!useHubShotOffset);
  }
}
