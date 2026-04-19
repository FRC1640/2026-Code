package frc.robot.constants;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.helpers.PoseFilter;
import frc.robot.util.helpers.PoseFilter.PoseFilterType;

public class FieldConstants {

  public enum Zone {
    ALLIANCE_ZONE, ENEMY_ZONE, NEUTRAL_ZONE
  }

  public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final VisionSystemSim visionSim = new VisionSystemSim("main");

  public static final VisionSystemSim getVisionSim() {
    return visionSim;
  }

  public static final Pose2d hubPositionBlue = new Pose2d(new Translation2d(4.625594, 4.0346376), Rotation2d.kZero);
  public static final Pose2d hubPositionRed = new Pose2d(new Translation2d(11.915394, 4.0346376), Rotation2d.kPi);

  public static final double fieldWidth = 16.540988;
  public static final double fieldHeight = 8.069326;

  public static final Pose2d[] blueBumpCenters = {new Pose2d(4.625594, 2.516886, new Rotation2d()),
      new Pose2d(4.625594, 5.5523892, new Rotation2d())};
  public static final Pose2d[] redBumpCenters = {new Pose2d(11.915394, 2.516886, new Rotation2d()),
      new Pose2d(11.915394, 5.5523892, new Rotation2d())};
  public static final Pose2d[] blueTrenchCenters = {new Pose2d(4.625594, 0.642493, new Rotation2d()),
      new Pose2d(4.625594, 7.4267822, new Rotation2d())};
  public static final Pose2d[] redTrenchCenters = {new Pose2d(11.915394, 0.642493, new Rotation2d()),
      new Pose2d(11.915394, 7.4267822, new Rotation2d())};
  public static final Pose2d[] allTrenchCenters = {FieldConstants.blueTrenchCenters[0],
      FieldConstants.blueTrenchCenters[1], FieldConstants.redTrenchCenters[0],
      FieldConstants.redTrenchCenters[1]};

  public static final double bumpLength = 1.82;
  public static final double bumpWidth = 1.1938;

  public static final double trenchLength = 1.281938;

  public static final Translation2d blueDepotCenter = new Translation2d(0, 5.9632596);
  public static final Translation2d redDepotCenter = new Translation2d(16.540988, 2.1060156);

  public static final Translation2d blueOutpostCenter = new Translation2d(0, 0.665988);
  public static final Translation2d redOutpostCenter = new Translation2d(16.540988, 7.4032872);

  public static final Translation2d blueTowerBarCenter = new Translation2d(1.139444, 3.7455856);
  public static final Translation2d redTowerBarCenter = new Translation2d(15.401544, 4.3236896);

  public static final double towerOutsidePost2PostWidth = 0.89535;
  public static final double depotWidth = 1.0668;

  public static final Pose2d blueShootDepot = new Pose2d(
      new Translation2d(hubPositionBlue.getX() - 1.943 - 0.854 - 0.8,
          hubPositionBlue.getY() + 1.602 + 1.520 - 0.762),
      Rotation2d.kZero);
  public static final Pose2d blueShootOutpost = new Pose2d(
      new Translation2d(hubPositionBlue.getX() - 1.943 - 0.854 - 0.8,
          hubPositionBlue.getY() - 1.602 - 1.520 + 0.762),
      Rotation2d.kZero);

  public static final Pose2d redShootOutpost = new Pose2d(
      new Translation2d(hubPositionRed.getX() + 1.943 + 0.854 + 0.8,
          hubPositionRed.getY() + 1.602 + 1.520 - 0.762),
      Rotation2d.kPi);

  public static final Pose2d redShootDepot = new Pose2d(new Translation2d(hubPositionRed.getX() + 1.943 + 0.854 + 0.8,
      hubPositionRed.getY() - 1.602 - 1.520 + 0.762), Rotation2d.kPi);

  public static final Pose2d redStealOutpost = new Pose2d(new Translation2d(fieldWidth / 2.0 + 2.5, fieldHeight - 0.9),
      Rotation2d.kZero);

  public static final Pose2d redStealDepot = new Pose2d(new Translation2d(fieldWidth / 2.0 + 2.5, 0.9), Rotation2d.kZero);

  public static final Pose2d blueStealOutpost = new Pose2d(new Translation2d(fieldWidth / 2.0 - 2.5, 0.9),
      Rotation2d.kZero);

  public static final Pose2d blueStealDepot = new Pose2d(new Translation2d(fieldWidth / 2.0 - 2.5, fieldHeight - 0.9),
      Rotation2d.kZero);

  public static final Pose2d[] redStealPoints = {redStealOutpost, redStealDepot};
  public static final Pose2d[] blueStealPoints = {blueStealOutpost, blueStealDepot};

  public static final Pose2d[] blueShootPoints = {blueShootDepot, blueShootOutpost};
  public static final Pose2d[] redShootPoints = {redShootOutpost, redShootDepot};

  public static final PoseFilter blueAllianceZone = new PoseFilter(PoseFilterType.LEFT, hubPositionBlue);
  public static final PoseFilter redAllianceZone = new PoseFilter(PoseFilterType.RIGHT, hubPositionRed);
  public static final PoseFilter neutralZone = new PoseFilter(PoseFilterType.RIGHT, hubPositionBlue)
      .addFilter(PoseFilterType.LEFT, hubPositionRed);

  // you never know when your on mars
  // we should add moon and sun to account for the tides
  public static final double gravityEarth = 9.80665;

  public static final PoseFilter rightBlueTrench = new PoseFilter(new Translation2d(1, 0),
      new Pose2d(4.125594, 0, new Rotation2d()))
          .addFilter(new Translation2d(-1, 0), new Pose2d(5.125594, 0, new Rotation2d()))
          .addFilter(new Translation2d(0, 1), new Pose2d(0, 0, new Rotation2d()))
          .addFilter(new Translation2d(0, -1), new Pose2d(0, 1.281938, new Rotation2d()));
  public static final PoseFilter leftBlueTrench = new PoseFilter(new Translation2d(1, 0),
      new Pose2d(4.125594, 0, new Rotation2d()))
          .addFilter(new Translation2d(-1, 0), new Pose2d(5.125594, 0, new Rotation2d()))
          .addFilter(new Translation2d(0, 1), new Pose2d(0, 6.7858132, new Rotation2d()))
          .addFilter(new Translation2d(0, -1), new Pose2d(0, 8.0677512, new Rotation2d()));
  public static final PoseFilter rightRedTrench = new PoseFilter(new Translation2d(1, 0),
      new Pose2d(11.318494, 0, new Rotation2d()))
          .addFilter(new Translation2d(-1, 0), new Pose2d(12.512294, 0, new Rotation2d()))
          .addFilter(new Translation2d(0, 1), new Pose2d(0, 1.606886, new Rotation2d()))
          .addFilter(new Translation2d(0, -1), new Pose2d(0, 3.426886, new Rotation2d()));
  public static final PoseFilter leftRedTrench = new PoseFilter(new Translation2d(1, 0),
      new Pose2d(11.318494, 0, new Rotation2d()))
          .addFilter(new Translation2d(-1, 0), new Pose2d(12.512294, 0, new Rotation2d()))
          .addFilter(new Translation2d(0, 1), new Pose2d(0, 4.6423892, new Rotation2d()))
          .addFilter(new Translation2d(0, -1), new Pose2d(0, 6.4623892, new Rotation2d()));
}
