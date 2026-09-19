package frc.robot.util.autoalign.controller.impl;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotConstants.AutopilotConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.autoalign.controller.IAlignController;
import frc.robot.util.autoalign.pointprovider.IAlignPointProvider;

public class AutopilotAlignController implements IAlignController {
  Pose2d goalPose = null;

  boolean isAligning = false;

  APTarget target;
  APResult out;
  PIDController controller;

  public AutopilotAlignController() {
    this.controller = RobotPIDConstants.constructPID(RobotPIDConstants.autopilotTurnPID);
    this.controller.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public ChassisSpeeds calculate(IAlignPointProvider pointProvider, ChassisSpeeds robotChassisSpeeds) {
    Pose2d alignToPose = pointProvider.getTargetPose();
    Pose2d robotPose = pointProvider.getRobotPose();
    target = new APTarget(alignToPose);
    if (!AutopilotConstants.kAutopilot.atTarget(robotPose, target) && pointProvider.hasPoint()) {
      isAligning = true;
      out = AutopilotConstants.kAutopilot.calculate(robotPose, robotChassisSpeeds, target);
      return new ChassisSpeeds(out.vx(), out.vy(), RadiansPerSecond
          .of(controller.calculate(robotPose.getRotation().getRadians(), out.targetAngle().getRadians())));
    } else if (AutopilotConstants.kAutopilot.atTarget(robotPose, target)) {
      pointProvider.onComplete();
    } else {
      isAligning = false;
    }
    return new ChassisSpeeds();
  }
  @Override
  public boolean isAligning() {
    return isAligning;
  }
}
