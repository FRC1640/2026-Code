package frc.robot.subsystems.shooter.turret;

import static java.lang.Math.PI;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.limits.Limits;

public class TurretConstants {
  public static final int canId = 16;

  public static final Transform3d turretTransform = new Transform3d(
      new Translation3d(-Units.inchesToMeters(5.8125), 0, 0), new Rotation3d(0, 0, 0)); // TODO
  public static final Transform2d turretTransform2d = new Transform2d(
      turretTransform.getTranslation().toTranslation2d(), turretTransform.getRotation().toRotation2d());
  public static final double turretZeroOffsetRobotFrame = PI / 2;
  // mechanical
  // please
  // save
  // us

  // limits
  public static final Limits turretAngleLimits = new Limits(-5 * PI / 6, 3 * PI / 4, true);
  // represents the negative slope of the trapezoidal velocity dropoff (greater
  // than 2, normalized onto 1x1 rectangle)
  public static final double velocityLimitRate = 4;

  public static final double potLowerVoltage = 0.333;
  public static final double potUpperVoltage = 3.041;
}
