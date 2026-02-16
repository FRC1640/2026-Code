package frc.robot.subsystems.drive.weights;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.RobotPIDConstants;

public class LockToPoint implements DriveWeight {
  public static final int X = 0, Y = 1;

  private final Vector<N3> weight;

  // TODO Tune
  Supplier<Pose2d> robotPose, robotTarget;
  PIDController drivePidX, drivePidY, rotPid;
  int lockTo;
  boolean lockRotation;

  public LockToPoint(Supplier<Pose2d> robotPose, Supplier<Pose2d> robotTarget, int lockTo, boolean lockRotation) {
    this.robotPose = robotPose;
    this.robotTarget = robotTarget;
    this.lockTo = lockTo;
    this.lockRotation = lockRotation;
    weight = VecBuilder.fill(lockTo == X ? 5 : 0, lockTo == Y ? 5 : 0, lockRotation ? 1 : 0);
    drivePidX = RobotPIDConstants.constructPID(RobotPIDConstants.autoDrivePID);
    drivePidY = RobotPIDConstants.constructPID(RobotPIDConstants.autoDrivePID);
    rotPid = RobotPIDConstants.constructPID(RobotPIDConstants.autoTurnPID);
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    return new ChassisSpeeds(drivePidX.calculate(robotPose.get().getX(), robotTarget.get().getX()),
        drivePidY.calculate(robotPose.get().getY(), robotTarget.get().getY()), rotPid.calculate(
            robotPose.get().getRotation().getRadians(), robotTarget.get().getRotation().getRadians()));
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
}
