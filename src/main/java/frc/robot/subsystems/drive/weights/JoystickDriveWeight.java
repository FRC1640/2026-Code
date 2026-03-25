package frc.robot.subsystems.drive.weights;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.helpers.DistanceManager;
import frc.robot.util.helpers.AllianceManager;

public class JoystickDriveWeight implements DriveWeight {
  private static final String name = "JoystickDriveWeight";

  private DoubleSupplier xPercent;
  private DoubleSupplier yPercent;
  private DoubleSupplier omegaPercent;
  private BooleanSupplier slowMode;
  private BooleanSupplier fastMode;
  private BooleanSupplier isFC;
  private Gyro gyro;
  private BooleanSupplier isLimited;
  private BooleanSupplier lockDirection;
  private int directionLockResolution;

  public JoystickDriveWeight(DoubleSupplier xPercent, DoubleSupplier yPercent, DoubleSupplier omegaPercent,
      BooleanSupplier slowMode, BooleanSupplier fastMode, BooleanSupplier isFC, Gyro gyro,
      BooleanSupplier isLimited) {
    this(xPercent, yPercent, omegaPercent, slowMode, fastMode, isFC, gyro, isLimited, () -> false, 4);
  }

  public JoystickDriveWeight(DoubleSupplier xPercent, DoubleSupplier yPercent, DoubleSupplier omegaPercent,
      BooleanSupplier slowMode, BooleanSupplier fastMode, BooleanSupplier isFC, Gyro gyro,
      BooleanSupplier isLimited, BooleanSupplier lockDirection, int directionLockResolution) {
    this.xPercent = xPercent;
    this.yPercent = yPercent;
    this.omegaPercent = omegaPercent;
    this.slowMode = slowMode;
    this.fastMode = fastMode;
    this.isFC = isFC;
    this.gyro = gyro;
    this.isLimited = isLimited;
    this.lockDirection = lockDirection;
    this.directionLockResolution = directionLockResolution;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    if (!(RobotState.isTeleop() || RobotState.isTest())) {
      return new ChassisSpeeds();
    }
    Translation2d linearVelocity = getLinearVelocityFromJoysticks(xPercent.getAsDouble(), yPercent.getAsDouble());
    if (lockDirection.getAsBoolean()) {
      linearVelocity = snapVelocityDirection(linearVelocity, directionLockResolution);
    }
    double omega = MathUtil.applyDeadband(omegaPercent.getAsDouble(), DriveConstants.driveControllerDeadband);
    omega = Math.copySign(omega * omega, omega);
    if (linearVelocity.getNorm() != 0 && linearVelocity.getNorm() > 1) {
      linearVelocity = linearVelocity.div(linearVelocity.getNorm());
    }
    omega = MathUtil.clamp(omega, -1, 1);
    double xyMult = 0.90;
    double omegaMult = 0.4;
    if (slowMode.getAsBoolean()) {
      xyMult = 0.3;
      omegaMult = 0.2;
    } else if (fastMode.getAsBoolean()) {
      xyMult = 0.99;
      omegaMult = 0.9;
    }
    double scale = isLimited.getAsBoolean() ? 0.45 : 1;
    ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX() * DriveConstants.maxSpeed * xyMult,
        linearVelocity.getY() * DriveConstants.maxSpeed * xyMult, omega * DriveConstants.maxOmega * omegaMult)
            .times(scale);

    if (!isFC.getAsBoolean()) {
      Translation2d speedsNotRotated = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

      speedsNotRotated = speedsNotRotated
          .rotateBy(gyro.getAngleRotation2d().plus(Rotation2d.fromRadians(Math.PI)));

      return new ChassisSpeeds(speedsNotRotated.getX(), speedsNotRotated.getY(), speeds.omegaRadiansPerSecond);
    }

    return speeds;
  }

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {

    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.driveControllerDeadband);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Flip direction for driver station perspective
    linearDirection = linearDirection.plus(AllianceManager.chooseFromAlliance(Rotation2d.kZero, Rotation2d.kPi));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();
  }

  private static Translation2d snapVelocityDirection(Translation2d linearVelocity, int snapResolution) {
    double velocityAngle = MathUtil.angleModulus(linearVelocity.getAngle().getRadians()) + Math.PI;
    double speed = linearVelocity.getNorm();
    double rotationStep = 2 * Math.PI / snapResolution;
    double lowerAngle = 0;
    for (int i = 0; i < rotationStep - 1; i++) {
      if (velocityAngle - lowerAngle <= rotationStep) {
        break;
      }
      lowerAngle += rotationStep;
    }
    double upperAngle = lowerAngle + rotationStep;
    double snappedAngle = DistanceManager.angleDistance(velocityAngle, lowerAngle) < DistanceManager
        .angleDistance(velocityAngle, upperAngle) ? lowerAngle : upperAngle;
    Logger.recordOutput("A_DEBUG/lowerAngle", lowerAngle);
    Logger.recordOutput("A_DEBUG/upperAngle", upperAngle);
    Logger.recordOutput("A_DEBUG/velocityAngle", velocityAngle);
    Logger.recordOutput("A_DEBUG/snappedAngle", snappedAngle);
    return new Translation2d(speed, new Rotation2d(snappedAngle));
  }

  @Override
  public Vector<N3> getWeight() {
    return VecBuilder.fill(1, 1, 1);
  }

  @Override
  public String getName() {
    return name;
  }
}
