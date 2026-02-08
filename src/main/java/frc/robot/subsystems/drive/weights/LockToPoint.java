package frc.robot.subsystems.drive.weights;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotPIDConstants;

public class LockToPoint implements DriveWeight {

  public static final int X = 0, Y = 1;
  // TODO Tune
  Supplier<Pose2d> robotPose, robotTarget;
  PIDController drivePID, rotPID;
  Supplier<Integer> lockTo;
  Supplier<Boolean> lockRotation;

  public LockToPoint(Supplier<Pose2d> robotPose, Supplier<Pose2d> robotTarget, Supplier<Integer> lockTo,
      Supplier<Boolean> lockRotation) {
    this.robotPose = robotPose;
    this.robotTarget = robotTarget;
    drivePID = RobotPIDConstants.constructPID(RobotPIDConstants.autoDrivePID);
    rotPID = RobotPIDConstants.constructPID(RobotPIDConstants.autoTurnPID);
    this.lockTo = lockTo;
    this.lockRotation = lockRotation;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    return new ChassisSpeeds(
        (lockTo.get() == X) ? drivePID.calculate(robotPose.get().getX(), robotTarget.get().getX()) : 0,
        (lockTo.get() == Y) ? drivePID.calculate(robotPose.get().getY(), robotTarget.get().getY()) : 0,
        lockRotation.get()
            ? rotPID.calculate(robotPose.get().getRotation().getRadians(),
                robotTarget.get().getRotation().getRadians())
            : 0);
  }

}
