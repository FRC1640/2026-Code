package frc.robot.util.autoalign.system.controller.impl;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.autoalign.system.controller.AlignControllerBase;
import frc.robot.util.autoalign.system.eventshandler.AlignSystemEventBus;
import frc.robot.util.autoalign.system.pointprovider.IAlignPointProvider;

public class LinearAlignController extends AlignControllerBase {
  private final PIDController TRANSLATION_PID;
  private final PIDController ROTATION_PID;

  private final SlewRateLimiter ACCEL_LIMITER;
  private final double maxAccelerationMetersPerSecondSquared = 3;

  private final AlignSystemEventBus EVENT_BUS = new AlignSystemEventBus();

  private final double OUTPUT_DEADBAND = 0.01;
  private final double TRANSLATION_DEADBAND = 0.02;
  private final double ROTATION_DEADBAND = Units.degreesToRadians(5);

  public LinearAlignController(IAlignPointProvider pointProvider) {
    super(pointProvider); // TODO fix align bus crash

    TRANSLATION_PID = RobotPIDConstants.constructPID(new PIDConstants(0.25, 0, 0));
    ROTATION_PID = RobotPIDConstants.constructPID(new PIDConstants(0.5, 0.001, 0.0001));
    
    ACCEL_LIMITER = new SlewRateLimiter(maxAccelerationMetersPerSecondSquared);
  }

  @Override
  public ChassisSpeeds calculate(ChassisSpeeds measuredSpeeds) {
    Pose2d robot = pointProvider.getRobotPose();
    Pose2d target = pointProvider.getTargetPose();

    Rotation2d angleToTarget = robot.getTranslation().minus(target.getTranslation()).getAngle();
    double dist = robot.getTranslation().getDistance(target.getTranslation());

    double linearOutput = TRANSLATION_PID.calculate(dist, 0);
    linearOutput = MathUtil.clamp(linearOutput, -1, 1);
    linearOutput = MathUtil.applyDeadband(linearOutput, OUTPUT_DEADBAND);
    linearOutput *= DriveConstants.maxSpeed;
    linearOutput = ACCEL_LIMITER.calculate(linearOutput);

    double rotationalOutput = ROTATION_PID.calculate(robot.getRotation().minus(target.getRotation()).getRadians(),
        0);
    rotationalOutput = MathUtil.clamp(rotationalOutput, -1, 1);
    rotationalOutput = MathUtil.applyDeadband(rotationalOutput, OUTPUT_DEADBAND);
    rotationalOutput *= DriveConstants.maxOmega;

    double xSpeed = Math.cos(angleToTarget.getRadians()) * linearOutput;
    double ySpeed = -Math.sin(angleToTarget.getRadians()) * linearOutput;

    return new ChassisSpeeds(xSpeed, ySpeed, rotationalOutput); // TODO coordinate system
  }

  @Override
  public boolean isAligning() { // TODO fire event when true (in periodic)
    return pointProvider.getRobotPose().getTranslation()
        .getDistance(pointProvider.getTargetPose().getTranslation()) < TRANSLATION_DEADBAND
        && Math.abs(pointProvider.getRobotPose().getRotation()
            .minus(pointProvider.getTargetPose().getRotation()).getRadians()) < ROTATION_DEADBAND;
  }

  @Override
  public AlignSystemEventBus getAlignSystemEventBus() {
    return EVENT_BUS;
  }
}
