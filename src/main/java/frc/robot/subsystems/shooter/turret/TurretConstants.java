package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.limits.Limits;
import static java.lang.Math.PI;

public class TurretConstants {
  public static final int canId = 16;

  public static final Transform2d turretTransform = new Transform2d(new Translation2d(), new Rotation2d()); // TODO
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
