package frc.robot.util.autoalign.system.controller.impl;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotConstants.AutopilotConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.autoalign.system.controller.IAlignController;
import frc.robot.util.autoalign.system.events.AlignSystemEventBus;
import frc.robot.util.autoalign.system.events.events.ECompleteReachPoint;
import frc.robot.util.autoalign.system.pointprovider.IAlignPointProvider;
import lombok.Getter;

public class AutopilotAlignController implements IAlignController {
  Pose2d goalPose = null;

  @Getter boolean isAligning = false;
  @Getter IAlignPointProvider pointProvider;

  APTarget target;
  APResult out;
  PIDController controller;

  AlignSystemEventBus alignSystemEventBus;
  public AutopilotAlignController(IAlignPointProvider pointProvider) {
    this.controller = RobotPIDConstants.constructPID(RobotPIDConstants.autopilotTurnPID);
    this.controller.enableContinuousInput(0, 2 * Math.PI);
    this.pointProvider = pointProvider;
    this.pointProvider.attachAlignController(this);
    alignSystemEventBus = new AlignSystemEventBus();
  }

  @Override
  public ChassisSpeeds calculate(ChassisSpeeds robotChassisSpeeds) {
    this.update();
    target = new APTarget(pointProvider.getTargetPose());
    if (!isComplete() && pointProvider.hasPoint()) {
      isAligning = true;
      out = AutopilotConstants.kAutopilot.calculate(pointProvider.getRobotPose(), robotChassisSpeeds, target);
      return new ChassisSpeeds(out.vx(), out.vy(), RadiansPerSecond
          .of(controller.calculate(pointProvider.getRobotPose().getRotation().getRadians(), out.targetAngle().getRadians())));
    } else {
      isAligning = false;
    }
    return new ChassisSpeeds();
  }

  void update() {
    if (isComplete())  {
      getAlignSystemEventBus().fireEvent(ECompleteReachPoint.class, new ECompleteReachPoint());
    }
  }

  boolean isComplete() {
    return AutopilotConstants.kAutopilot.atTarget(pointProvider.getRobotPose(), target);
  }
  
  @Override
  public AlignSystemEventBus getAlignSystemEventBus() {
    return alignSystemEventBus;
  }
}
