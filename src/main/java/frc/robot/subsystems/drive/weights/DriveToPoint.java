package frc.robot.subsystems.drive.weights;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotPIDConstants;

public class DriveToPoint implements DriveWeight {

  // TODO Tune
  Supplier<Pose2d> robotPose, robotTarget;
  PIDController drivePidX, drivePidY, rotPID;

  public DriveToPoint(Supplier<Pose2d> robotPose, Supplier<Pose2d> robotTarget) {
    this.robotPose = robotPose;
    this.robotTarget = robotTarget;
    drivePidX = RobotPIDConstants.constructPID(RobotPIDConstants.autoDrivePidX);
    drivePidY = RobotPIDConstants.constructPID(RobotPIDConstants.autoDrivePidY);
    rotPID = RobotPIDConstants.constructPID(RobotPIDConstants.autoTurnPID);
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    Logger.recordOutput("DriveToPoint/target", robotTarget.get());
    return new ChassisSpeeds(drivePidX.calculate(robotPose.get().getX(), robotTarget.get().getX()),
        drivePidY.calculate(robotPose.get().getY(), robotTarget.get().getY()), rotPID.calculate(
            robotPose.get().getRotation().getRadians(), robotTarget.get().getRotation().getRadians()));

  }

}
