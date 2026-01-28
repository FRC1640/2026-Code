package frc.robot.constants;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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

  public static final Translation2d[] blueBumpCenters = {new Translation2d(4.625594,2.516886), new Translation2d(4.625594,5.5523892)};
  public static final Translation2d[] redBumpCenters = {new Translation2d(11.915394,2.516886), new Translation2d(11.915394,5.5523892)};
  public static final Translation2d[] blueTrenchCenters = {new Translation2d(4.625594,0.642493), new Translation2d(4.625594,7.4267822)};
  public static final Translation2d[] redTrenchCenters = {new Translation2d(11.915394,0.642493), new Translation2d(11.915394,7.4267822)};

  public static final double bumpLength = 1.82;
  public static final double bumpWidth = 1.1938;

  public static final double trenchLength = 1.281938;

  public static final Translation2d blueDepotCenter = new Translation2d(0,5.9632596);
  public static final Translation2d redDepotCenter = new Translation2d(16.540988,2.1060156);

  public static final Translation2d blueOutpostCenter = new Translation2d(0,0.665988);
  public static final Translation2d redOutpostCenter = new Translation2d(16.540988,7.4032872);

  public static final Translation2d blueTowerBarCenter = new Translation2d(1.139444,3.7455856);
  public static final Translation2d redTowerBarCenter = new Translation2d(15.401544,4.3236896);
}
