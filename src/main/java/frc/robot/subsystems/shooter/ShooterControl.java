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
      double flywheelVelocity) {
  }

  private TurretSetpoint setpoint;
  private TurretSetpoint lastSetpoint;

  private static final InterpolatingDoubleTreeMap distanceToDeflectorAngle = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap distanceToFlywheelVelocity = new InterpolatingDoubleTreeMap();

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
    hubTags.put(AllianceManager.chooseFromAlliance(25, 9),
        AllianceManager.chooseFromAlliance(
            FieldConstants.hubPositionBlue
                .minus(FieldConstants.aprilTagLayout.getTagPose(25).get().toPose2d()).getTranslation(),
            FieldConstants.hubPositionRed
                .minus(FieldConstants.aprilTagLayout.getTagPose(9).get().toPose2d()).getTranslation()));
    hubTags.put(AllianceManager.chooseFromAlliance(26, 10), AllianceManager.chooseFromAlliance(
        FieldConstants.hubPositionBlue.minus(FieldConstants.aprilTagLayout.getTagPose(26).get().toPose2d())
            .getTranslation(),
        FieldConstants.hubPositionRed.minus(FieldConstants.aprilTagLayout.getTagPose(10).get().toPose2d())
            .getTranslation()));
    for (int id : hubTags.keySet()) {
      turretCamera.addTrackingId(id);
    }
    setpoint = new TurretSetpoint(0, 0, 0, 0);
    lastSetpoint = new TurretSetpoint(0, 0, 0, 0);
    ShooterControl.instance = this;
    Logger.recordOutput("Shooter/tag25", FieldConstants.aprilTagLayout.getTagPose(25).get());
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

    Translation2d planarProjectileVelocity = new Translation2d(flywheelVelocity * Math.cos(deflectorAngle),
        targetOffset.getAngle()); // fieldcentric

    planarProjectileVelocity = planarProjectileVelocity.minus(turretVelocity); // fieldcentric, compensated for
    // moving

    flywheelVelocity *= (planarProjectileVelocity.getNorm() / (flywheelVelocity * Math.cos(deflectorAngle)));

    TurretSetpoint output = new TurretSetpoint(turretAngle, (turretAngle - lastSetpoint.turretAngle()) / 0.02,
        deflectorAngle, flywheelVelocity);

    lastSetpoint = setpoint;
    setpoint = output;

    Logger.recordOutput("Shooter/turretPose", turretPose);
    Logger.recordOutput("Shooter/targetOffset", targetOffset);
    Logger.recordOutput("Shooter/turretTargeting", robotPose.get()
        .plus(new Transform2d(new Translation2d(1, new Rotation2d(turretAngle)), new Rotation2d())));
    Logger.recordOutput("Shooter/angleToTarget",
        targetOffset.getNorm() != 0 ? targetOffset.getAngle() : new Rotation2d());
    Logger.recordOutput("Shooter/robotRotation", robotPose.get().getRotation());
    Logger.recordOutput("Shooter/planarProjectileVelocity", robotPose.get().plus(new Transform2d(planarProjectileVelocity, new Rotation2d())));
    return setpoint;
  }
}
