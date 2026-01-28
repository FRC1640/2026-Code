package frc.robot.subsystems.drive.weights;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotPIDConstants;

public class DriveToPoint implements DriveWeight {

    // TODO Tune
    Supplier<Pose2d> robotPose, robotTarget;
    Supplier<Boolean> enabled;
    PIDController drivePID, rotPID;

    public DriveToPoint(Supplier<Pose2d> robotPose, Supplier<Pose2d> robotTarget, Supplier<Boolean> enabled) {
        this.robotPose = robotPose;
        this.robotTarget = robotTarget;
        this.enabled = enabled;
        drivePID = RobotPIDConstants.constructPID(RobotPIDConstants.autoDrivePID);
        rotPID = RobotPIDConstants.constructPID(RobotPIDConstants.autoTurnPID);
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        if (!enabled.get()) {
            return new ChassisSpeeds();
        } else {
            return new ChassisSpeeds(
                    drivePID.calculate(robotPose.get().getX(), robotTarget.get().getX()),
                    drivePID.calculate(robotPose.get().getY(), robotTarget.get().getY()),
                    rotPID.calculate(robotPose.get().getRotation().getRadians(), robotTarget.get().getRotation().getRadians()));
        }

    }

}
