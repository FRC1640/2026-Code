package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.RobotTypes;
import frc.robot.subsystems.module.ModuleInfo;
import frc.robot.util.robotswitcher.Switchable;
import frc.robot.util.robotswitcher.SwitchableCANID;
// TODO update for new robot
public class DriveConstants {

  /**
   * Represents a swerve module on the robot.
   */
  public static enum PivotId {
    FL, FR, BL, BR;
  }

  public static final double driveControllerDeadband = 0.03;

  public static final double wheelYPos = Units.inchesToMeters(20.25 / 2);
  public static final double wheelXPos = Units.inchesToMeters(20.25 / 2);

  // Module translations
  public static final Translation2d frontLeftLocation = new Translation2d(wheelXPos, wheelYPos);
  public static final Translation2d frontRightLocation = new Translation2d(wheelXPos, -wheelYPos);
  public static final Translation2d backLeftLocation = new Translation2d(-wheelXPos, wheelYPos);
  public static final Translation2d backRightLocation = new Translation2d(-wheelXPos, -wheelYPos);

  public static final Translation2d[] positions = new Translation2d[]{frontLeftLocation, frontRightLocation,
      backLeftLocation, backRightLocation};

  public static double odometryFrequency = 200.0;

  // Gear ratios
  public static final double driveGearRatio = 116.0 / 15.0;
  public static final double steerGearRatio = ((480.0 / 11.0)) * 1.0166667 * 0.99790377777778;

  // Speeds
  public static final double maxSpeed = 4.25;
  public static final double maxNorm = DriveSubsystem.computeMaxNorm(DriveConstants.positions, new Translation2d());
  public static final double maxOmega = (maxSpeed / maxNorm);
  public static final double accelLimit = 20;
  public static final double deaccelLimit = 11;

  public static final double wheelRadius = Units.inchesToMeters(2);

  public static final double initalSlope = 3.125;
  public static final double finalSlope = 4.375;

  public static final double maxSteerSpeed = 50; // rad per second

  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation,
      frontRightLocation, backLeftLocation, backRightLocation);

  public static final ModuleInfo FL = new ModuleInfo(PivotId.FL, SwitchableCANID.of(1).get(),
      SwitchableCANID.of(2).get(),
      Switchable.of(0).addAlt(RobotTypes.frank25, 2).addAlt(RobotTypes.prime25, 2).get(), 45);

  public static final ModuleInfo FR = new ModuleInfo(PivotId.FR, SwitchableCANID.of(3).get(),
      SwitchableCANID.of(4).get(),
      Switchable.of(1).addAlt(RobotTypes.frank25, 0).addAlt(RobotTypes.prime25, 0).get(), -45);

  public static final ModuleInfo BL = new ModuleInfo(PivotId.BL,
      SwitchableCANID.of(7).addAlt(RobotConstants.RobotTypes.prime25, 5).get(), SwitchableCANID.of(8).get(),
      Switchable.of(2).addAlt(RobotTypes.frank25, 1).addAlt(RobotTypes.prime25, 1).get(), 135);

  public static final ModuleInfo BR = new ModuleInfo(PivotId.BR,
      SwitchableCANID.of(5).addAlt(RobotConstants.RobotTypes.prime25, 7).get(), SwitchableCANID.of(6).get(),
      Switchable.of(3).get(), -135);

  public static final double maxAntiTipCorrectionSpeed = 1.5;
  public static final double minTipDegrees = 3;
}
