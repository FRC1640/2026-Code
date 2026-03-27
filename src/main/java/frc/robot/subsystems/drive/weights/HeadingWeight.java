package frc.robot.subsystems.drive.weights;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.RobotPIDConstants;

public class HeadingWeight implements DriveWeight {
  private static final String name = "HeadingWeight";

  // TODO Tune
  Supplier<Pose2d> robotPose;
  Supplier<ChassisSpeeds> currentSpeeds;
  PIDController rotPID;
  final Vector<N3> weight;

  public HeadingWeight(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> currentSpeeds) {
    this.weight = VecBuilder.fill(0,0,1);
    this.robotPose = robotPose;
    this.currentSpeeds = currentSpeeds;
    rotPID = RobotPIDConstants.constructPID(RobotPIDConstants.autoTurnPID);
    rotPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    Translation2d fieldRelativeVelocity = new Translation2d(currentSpeeds.get().vxMetersPerSecond,
        currentSpeeds.get().vyMetersPerSecond).rotateBy(robotPose.get().getRotation());
    return new ChassisSpeeds(0, 0,
        fieldRelativeVelocity.getNorm() >= 0.04
            ? rotPID.calculate(robotPose.get().getRotation().getRadians(),
                (Math.atan2(fieldRelativeVelocity.getY(), fieldRelativeVelocity.getX())))
            : 0);
  }

  @Override
  public Vector<N3> getWeight(){
    return this.weight;
  }

  @Override
  public String getName() {
    return name;
  }

}
