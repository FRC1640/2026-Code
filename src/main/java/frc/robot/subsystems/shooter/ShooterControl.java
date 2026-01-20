package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import frc.robot.subsystems.shooter.turret.TurretConstants;

import frc.robot.util.periodic.PeriodicBase;

public class ShooterControl extends PeriodicBase {
  private Supplier<Pose2d> robotPose;
  private Supplier<Translation2d> robotVelocity;
  private Supplier<Double> robotOmega;
  private Supplier<Pose2d> targetPose;

  private static ShooterControl instance;

  public static record TurretSetpoint(double turretAngle, double turretOmega, double hoodAngle,
      double flywheelSpeed) {
  }

  public TurretSetpoint setpoint;
  private TurretSetpoint lastSetpoint;

  private static final InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToFlywheelSpeed = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToTimeOfFlight = new InterpolatingDoubleTreeMap();

  static {
    // TODO initialize lookup tables
  }

  public ShooterControl(Supplier<Pose2d> robotPose, Supplier<Translation2d> robotVelocity,
      Supplier<Double> robotOmega, Supplier<Pose2d> targetPose) {
    this.robotPose = robotPose;
    this.robotVelocity = robotVelocity;
    this.robotOmega = robotOmega;
    this.targetPose = targetPose;
    ShooterControl.instance = this;
  }

  @Override
  public void periodic() {

    // calculate turret velocity
    Translation2d turretOffset = TurretConstants.turretTransform.getTranslation();
    Translation2d turretVelocity = robotVelocity.get()
        .plus(turretOffset.rotateBy(Rotation2d.kCCW_Pi_2).times(turretOffset.getNorm() * robotOmega.get()));

    // calculate distance to target
    Pose2d turretPose = robotPose.get().plus(TurretConstants.turretTransform);
    Translation2d targetOffset = targetPose.get().minus(turretPose).getTranslation();

    // calculate distance to adjusted target accounting for robot velocity
    Translation2d deltaR = turretVelocity.times(distanceToTimeOfFlight.get(targetOffset.getNorm()));
    Translation2d adjustedDistance = targetOffset.minus(deltaR);

    // use lookup tables to get hood angle and flywheel speed
    double hoodAngle = distanceToHoodAngle.get(adjustedDistance.getNorm());
    double flywheelSpeed = distanceToFlywheelSpeed.get(adjustedDistance.getNorm());

    // calculate turret angle setpoint
    double turretAngle = targetOffset.getAngle().minus(robotPose.get().getRotation()).getRadians();
    lastSetpoint = setpoint;
    setpoint = new TurretSetpoint(turretAngle, (turretAngle - lastSetpoint.turretAngle()) / 0.02, hoodAngle,
        flywheelSpeed);
  }
}
