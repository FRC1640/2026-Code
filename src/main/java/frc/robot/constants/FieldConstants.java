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

  public static final Pose2d blueShootNorth = new Pose2d(new Translation2d(3.30, fieldHeight - 1.10),
      Rotation2d.kZero);
  public static final Pose2d blueShootSouth = new Pose2d(new Translation2d(3.30, 1.10), Rotation2d.kZero);

  public static final Pose2d redShootNorth = new Pose2d(new Translation2d(fieldWidth - 3.30, fieldHeight - 1.10),
      Rotation2d.kPi);

  public static final Pose2d redShootSouth = new Pose2d(new Translation2d(fieldWidth - 3.30, 1.10), Rotation2d.kPi);

  public static final Pose2d neutralShootNorth = new Pose2d(new Translation2d(fieldWidth / 2.0, fieldHeight - 1.10),
      Rotation2d.kZero);

  public static final Pose2d neutralShootSouth = new Pose2d(new Translation2d(fieldWidth / 2.0, 1.10),
      Rotation2d.kZero);

  public static final Pose2d[] neutralShootPoints = {neutralShootNorth, neutralShootSouth};

  public static final Pose2d[] blueShootPoints = {blueShootNorth, blueShootSouth};
  public static final Pose2d[] redShootPoints = {redShootNorth, redShootSouth};

  public static final PoseFilter shooterTypeBluePoseFilter = new PoseFilter(PoseFilterType.LEFT, hubPositionBlue);
  public static final PoseFilter shooterTypeRedPoseFilter = new PoseFilter(PoseFilterType.RIGHT, hubPositionRed);

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
