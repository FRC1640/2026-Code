package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    Translation2d turretOffset = TurretConstants.turretTransform.getTranslation();
    Translation2d turretVelocity =
      robotVelocity.get()
        .plus(turretOffset.rotateBy(Rotation2d.kCCW_Pi_2)
        .times(TurretConstants.turretTransform.getTranslation().getNorm() * robotOmega.get() / turretOffset.getNorm()));
    double distToTarget = targetPose.get().minus(robotPose.get()).getTranslation().getNorm();
    
  }
}
