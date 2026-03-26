package frc.robot.subsystems.drive.weights;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotPIDConstants;

public class HeadingWeight implements DriveWeight {
  private static final String name = "HeadingWeight";

  // TODO Tune
  Supplier<Pose2d> robotPose;
  Supplier<ChassisSpeeds> currentSpeeds;
  PIDController rotPID;

  public HeadingWeight(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> currentSpeeds) {
    this.robotPose = robotPose;
    this.currentSpeeds = currentSpeeds;
    rotPID = RobotPIDConstants.constructPID(RobotPIDConstants.autoTurnPID);
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    Translation2d fieldRelativeVelocity = new Translation2d(currentSpeeds.get().vxMetersPerSecond,
        currentSpeeds.get().vyMetersPerSecond).rotateBy(robotPose.get().getRotation());
    return new ChassisSpeeds(0, 0,
        fieldRelativeVelocity.getNorm() >= 0.04
            ? rotPID.calculate((robotPose.get().getRotation().getRadians()-(Math.atan2(fieldRelativeVelocity.getY(), fieldRelativeVelocity.getX()))+Math.PI)%(2*Math.PI) - Math.PI,
                0)
            : 0);

  }

  @Override
  public String getName() {
    return name;
  }

}
