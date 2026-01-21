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
      double flywheelSpeed) {
  }

  public TurretSetpoint setpoint;
  private TurretSetpoint lastSetpoint;

  private static final InterpolatingDoubleTreeMap distanceToDeflectorAngle = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToFlywheelSpeed = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToTimeOfFlight = new InterpolatingDoubleTreeMap();

  static {
    // TODO initialize lookup tables
  }

  public ShooterControl(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotVelocity, Supplier<Pose2d> targetPose) {
    this.robotPose = robotPose;
    this.robotVelocity = robotVelocity;
    this.targetPose = targetPose;
    ShooterControl.instance = this;
  }

  public static ShooterControl getInstance() {
    return instance;
  }

  public static void clearSetpoint() {
    getInstance().setpoint = null;
  }

  public TurretSetpoint getSetpoint() {

    if (setpoint != null) {
      return setpoint;
    }

    // calculate turret velocity
    Translation2d turretOffset = TurretConstants.turretTransform.getTranslation();
    ChassisSpeeds velocity = robotVelocity.get();
    Translation2d turretVelocity = new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond)
        .plus(turretOffset.rotateBy(Rotation2d.kCCW_Pi_2).times(turretOffset.getNorm() * velocity.omegaRadiansPerSecond));

    // calculate distance to target
    Pose2d turretPose = robotPose.get().plus(TurretConstants.turretTransform);
    Translation2d targetOffset = targetPose.get().minus(turretPose).getTranslation();

    // calculate distance to adjusted target accounting for robot velocity
    Translation2d deltaR = turretVelocity.times(distanceToTimeOfFlight.get(targetOffset.getNorm()));
    Translation2d adjustedDistance = targetOffset.minus(deltaR);

    // use lookup tables to get hood angle and flywheel speed
    double flywheelSpeed = distanceToFlywheelSpeed.get(adjustedDistance.getNorm());
    double deflectorAngle = distanceToDeflectorAngle.get(adjustedDistance.getNorm());

    // calculate turret angle setpoint
    double turretAngle = targetOffset.getAngle().minus(robotPose.get().getRotation()).getRadians();
    lastSetpoint = setpoint;

    setpoint = new TurretSetpoint(turretAngle, (turretAngle - lastSetpoint.turretAngle()) / 0.02, deflectorAngle,
        flywheelSpeed);
    return setpoint;
  }
}
