package frc.robot.subsystems.turret;

import static java.lang.Math.PI;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotConstants.RobotTypes;
import frc.robot.util.limits.Limits;
import frc.robot.util.robotswitcher.Switchable;
import frc.robot.util.robotswitcher.SwitchableCANID;

public class TurretConstants {
  public static final int canId = SwitchableCANID.of(9).addAlt(RobotTypes.frank25, 16).get();

  public static final Transform3d turretTransform = Switchable
      .of(new Transform3d(new Translation3d(-Units.inchesToMeters(5.863), -Units.inchesToMeters(6.063), 0),
          new Rotation3d(0, 0, Math.PI)))
      .addAlt(RobotTypes.frank25,
          new Transform3d(new Translation3d(-Units.inchesToMeters(5.8125), 0, 0), new Rotation3d(0, 0, 0)))
      .get();
  public static final Transform2d turretTransform2d = new Transform2d(
      turretTransform.getTranslation().toTranslation2d(), turretTransform.getRotation().toRotation2d());

  public static final double turretZeroOffsetRobotFrame = Switchable.of(PI).addAlt(RobotTypes.frank25, PI / 2).get();
  // mechanical
  // please
  // save
  // us

  // limits
  public static final Limits turretAngleLimits = Switchable
      .of(new Limits(Units.degreesToRadians(-130), Units.degreesToRadians(135), true))
      .addAlt(RobotTypes.frank25, new Limits(-5 * PI / 6, 3 * PI / 4, true)).get();

  // represents the slope of the trapezoidal velocity dropoff (greater
  // than 2, normalized onto 1x1 rectangle)
  public static final double velocityLimitRate = 4;

  public static final double turretAngle1Radians = Math.PI / 2;
  public static final double turretAngle0Radians = 0;
  public static final double potVoltage1 = 0.92;
  public static final double potVoltage0 = 1.575;

  public static final double turretPotToRadiansConversion = (turretAngle1Radians - turretAngle0Radians)
      / (potVoltage1 - potVoltage0);

  public static final double disconnectMinMotorVelocity = 10;
  public static final double disconnectMinPotVelocity = 0.01;
  // TODO: tune values^^^

  public static final double maxVelocityRadPerSec = 1.5 * PI;
}
