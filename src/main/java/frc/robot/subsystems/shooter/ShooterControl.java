package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.shooter.turret.TurretConstants;

public class ShooterControl {
  private Supplier<Pose2d> robotPose;
  private Supplier<ChassisSpeeds> robotVelocity;
  private Supplier<Pose2d> targetPose;

  private static ShooterControl instance;

  public static record TurretSetpoint(double turretAngle, double turretOmega, double hoodAngle,
      double flywheelVelocity) {
  }

  private TurretSetpoint setpoint;
  private TurretSetpoint lastSetpoint;

  private static final InterpolatingDoubleTreeMap distanceToDeflectorAngle = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToFlywheelVelocity = new InterpolatingDoubleTreeMap();

  static {
    // distance (m) -> deflector angle (deg)
    distanceToDeflectorAngle.put(1.5, 58.0);
    distanceToDeflectorAngle.put(2.0, 52.0);
    distanceToDeflectorAngle.put(2.5, 47.0);
    distanceToDeflectorAngle.put(3.0, 43.0);
    distanceToDeflectorAngle.put(3.5, 40.0);
    distanceToDeflectorAngle.put(4.0, 37.0);
    distanceToDeflectorAngle.put(4.5, 35.0);
    distanceToDeflectorAngle.put(5.0, 33.0);
    distanceToDeflectorAngle.put(5.5, 31.0);
    // custom format
                                                              // TODO: THESE ARE DUMMY VALUES!!!!!!!!
                                                              // spotless format
    // distance (m) -> flywheel surface RPM
    distanceToFlywheelVelocity.put(1.5, 3200.0);
    distanceToFlywheelVelocity.put(2.0, 3400.0);
    distanceToFlywheelVelocity.put(2.5, 3600.0);
    distanceToFlywheelVelocity.put(3.0, 3800.0);
    distanceToFlywheelVelocity.put(3.5, 4000.0);
    distanceToFlywheelVelocity.put(4.0, 4200.0);
    distanceToFlywheelVelocity.put(4.5, 4400.0);
    distanceToFlywheelVelocity.put(5.0, 4600.0);
    distanceToFlywheelVelocity.put(5.5, 4800.0);
  }

  public ShooterControl(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotVelocity, Supplier<Pose2d> targetPose) {
    this.robotPose = robotPose;
    this.robotVelocity = robotVelocity;
    this.targetPose = targetPose;
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

  public TurretSetpoint getSetpoint() {
    // sync logic
    if (setpoint != null) {
      return setpoint;
    }

    // current robot velocity and turret velocity
    ChassisSpeeds velocity = robotVelocity.get();

    Pose2d turretPose = robotPose.get().plus(TurretConstants.turretTransform2d); // fieldcentric

    // calculate distance to target
    Translation2d targetOffset = targetPose.get().getTranslation().minus(turretPose.getTranslation()); // fieldcentric

    double flywheelVelocity = distanceToFlywheelVelocity.get(targetOffset.getNorm()); // without compensation

    double deflectorAngle = distanceToDeflectorAngle.get(targetOffset.getNorm());

    Translation2d turretVelocity = turretPose.getTranslation().minus(robotPose.get().getTranslation()) // fieldcentric
        .rotateBy(Rotation2d.kCCW_Pi_2).times(velocity.omegaRadiansPerSecond)
        .plus(new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond));

    // use lookup tables to get hood angle and flywheel speed

    // calculate turret angle setpoint
    double turretAngle = targetOffset.getNorm() != 0 // robotcentric
        ? targetOffset.getAngle()
            .minus(robotPose.get().getRotation()
                .plus(new Rotation2d(TurretConstants.turretZeroOffsetRobotFrame)))
            .getRadians()
        : 0;

    Translation2d planarProjectileVelocity = new Translation2d(
        flywheelVelocity * Math.cos(Math.toRadians(deflectorAngle)), targetOffset.getAngle()); // fieldcentric

    planarProjectileVelocity = planarProjectileVelocity.minus(turretVelocity); // fieldcentric, compensated for
    // moving

    flywheelVelocity *= (planarProjectileVelocity.getNorm()
        / (flywheelVelocity * Math.cos(Math.toRadians(deflectorAngle))));

    TurretSetpoint output = new TurretSetpoint(turretAngle, (turretAngle - lastSetpoint.turretAngle()) / 0.02,
        deflectorAngle, flywheelVelocity);

    lastSetpoint = setpoint;
    setpoint = output;

    return setpoint;
  }
}
