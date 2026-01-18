package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.turret.TurretIO.TurretSetpoint;
import frc.robot.util.periodic.PeriodicBase;

public class TurretControl extends PeriodicBase {
  private Supplier<Pose2d> robotPose;
  private Supplier<Translation2d> robotVelocity;
  private Supplier<Double> robotOmega;
  private Supplier<Pose2d> targetPose;

  private static TurretControl instance;

  public TurretSetpoint setpoint;
  private TurretSetpoint lastSetpoint;

  private static final InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToFlywheelSpeed = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToTimeOfFlight = new InterpolatingDoubleTreeMap();

  static {
    // TODO initialize lookup tables
  }

  public TurretControl(Supplier<Pose2d> robotPose,
      Supplier<Translation2d> robotVelocity,
      Supplier<Double> robotOmega,
      Supplier<Pose2d> targetPose) {
    this.robotPose = robotPose;
    this.robotVelocity = robotVelocity;
    this.robotOmega = robotOmega;
    this.targetPose = targetPose;
    TurretControl.instance = this;
  }

  @Override
  public void periodic() {
    Pose2d turretPose = robotPose.get().plus(TurretConstants.turretTransform);

    //calculate turret velocity
    Translation2d turretOffset = TurretConstants.turretTransform.getTranslation();
    Translation2d turretVelocity =
      robotVelocity.get()
        .plus(turretOffset.rotateBy(Rotation2d.kCCW_Pi_2)
        .times(turretOffset.getNorm() * robotOmega.get() / turretOffset.getNorm()));

    // calculate distance to target
    Translation2d targetOffset = targetPose.get().minus(turretPose).getTranslation();
    double distToTarget = targetOffset.getNorm();

    // calculate adjusted distance to target accounting for robot velocity
    double deltaR = distanceToTimeOfFlight.get(distToTarget) * (targetOffset.div(distToTarget)).dot(turretVelocity);
    double adjustedDistance = distToTarget + deltaR;

    // use lookup tables to get hood angle and flywheel speed
    double hoodAngle = distanceToHoodAngle.get(adjustedDistance);
    double flywheelSpeed = distanceToFlywheelSpeed.get(adjustedDistance);
    
    // calculate turret angle setpoint
    double turretAngle = targetOffset.getAngle().minus(robotPose.get().getRotation()).getRadians();
    lastSetpoint = setpoint;
    setpoint = new TurretSetpoint(turretAngle, (turretAngle - lastSetpoint.turretAngle()) / 0.02, hoodAngle, flywheelSpeed);
  }
}
