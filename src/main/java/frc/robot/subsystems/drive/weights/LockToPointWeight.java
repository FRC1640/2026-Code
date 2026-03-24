package frc.robot.subsystems.drive.weights;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.helpers.DistanceManager;

public class LockToPointWeight implements DriveWeight {
  private static final String name = "LockToPointWeight";

  public static final int X = 0, Y = 1;
  public static final double activeDistanceX = 3, activeDistanceY = 4.5;
  private static final double baseLockWeight = 16;

  // TODO Tune
  private Supplier<Pose2d> robotPose, robotTarget;
  private PIDController drivePidX, drivePidY, rotPid;
  private double rotationInterval;

  private final Vector<N3> weight;

  public LockToPointWeight(Supplier<Pose2d> robotPose, Supplier<Pose2d> robotTarget, int lockTo,
      boolean lockRotation) {
    this.robotPose = robotPose;
    this.robotTarget = robotTarget;
    weight = VecBuilder.fill(lockTo == X ? baseLockWeight : 0, lockTo == Y ? baseLockWeight : 0,
        lockRotation ? baseLockWeight : 0);
    drivePidX = RobotPIDConstants.constructPID(RobotPIDConstants.autoDrivePID);
    drivePidY = RobotPIDConstants.constructPID(RobotPIDConstants.autoDrivePID);
    rotPid = RobotPIDConstants.constructPID(RobotPIDConstants.autoTurnPID);
    rotPid.enableContinuousInput(-Math.PI, Math.PI);
    rotationInterval = 2 * Math.PI;
  }

  public LockToPointWeight(Supplier<Pose2d> robotPose, Supplier<Pose2d> robotTarget, int lockTo,
      double rotationInterval) {
    this.robotPose = robotPose;
    this.robotTarget = robotTarget;
    weight = VecBuilder.fill(lockTo == X ? baseLockWeight : 0, lockTo == Y ? baseLockWeight : 0, baseLockWeight);
    drivePidX = RobotPIDConstants.constructPID(RobotPIDConstants.autoDrivePID);
    drivePidY = RobotPIDConstants.constructPID(RobotPIDConstants.autoDrivePID);
    rotPid = RobotPIDConstants.constructPID(RobotPIDConstants.autoTurnPID);
    rotPid.enableContinuousInput(-Math.PI, Math.PI);
    this.rotationInterval = rotationInterval;
  }

  public LockToPointWeight(Supplier<Pose2d> robotPose, Pose2d[] robotTargets, int lockTo, boolean lockRotation) {
    this(robotPose, () -> DistanceManager.getNearestPosition(robotPose.get(), robotTargets), lockTo, lockRotation);
  }

  public LockToPointWeight(Supplier<Pose2d> robotPose, Pose2d[] robotTargets, int lockTo, double rotationInterval) {
    this(robotPose, () -> DistanceManager.getNearestPosition(robotPose.get(), robotTargets), lockTo,
        rotationInterval);
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    // calculate linear velocity
    double vx = drivePidX.calculate(robotPose.get().getX(), robotTarget.get().getX());
    double vy = drivePidY.calculate(robotPose.get().getY(), robotTarget.get().getY());

    // calculate nearest angle setpoint
    double angle = robotPose.get().getRotation().getRadians();
    double angleSetpoint = robotTarget.get().getRotation().getRadians();
    double testAngle = angleSetpoint;
    // find minimum angle within interval
    for (int i = 0; i < 20; i++) {
      if (testAngle - rotationInterval <= -Math.PI)
        break;
      testAngle -= rotationInterval;
    }
    double delta = Math.min(Math.abs(angle - testAngle), Math.abs(angle - (testAngle - 2 * Math.PI)));
    double minDelta = delta;
    double minDeltaAngle = testAngle;
    // step up gradually to minimize delta
    for (int i = 0; i < 20; i++) {
      if (testAngle > Math.PI) {
        angleSetpoint = minDeltaAngle;
      }
      if (delta < minDelta) {
        minDelta = delta;
        minDeltaAngle = testAngle;
      }
      testAngle += rotationInterval;
      delta = Math.min(Math.abs(angle - testAngle), Math.abs(angle - (testAngle - 2 * Math.PI)));
    }
    // calculate rotational velocity
    double omega = rotPid.calculate(angle, angleSetpoint);

    Logger.recordOutput("LockToPoint/vx", vx);
    Logger.recordOutput("LockToPoint/vy", vy);
    Logger.recordOutput("LockToPoint/omega", omega);

    return new ChassisSpeeds(vx, vy, omega);
  }

  @Override
  public Vector<N3> getWeight() {
    return weight;
  }

  public Pose2d getTargetPoint() {
    return robotTarget.get();
  }

  public Pose2d getRobotPose() {
    return robotPose.get();
  }

  @Override
  public String getName() {
    return name;
  }
}
