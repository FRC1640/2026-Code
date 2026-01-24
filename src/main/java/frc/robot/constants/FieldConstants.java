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

  public final Pose2d hubPositionBlue = new Pose2d(new Translation2d(4.611399, 4.021132), Rotation2d.kZero);
  public final Pose2d hubPositionRed = new Pose2d(new Translation2d(11.900843, 4.021132), Rotation2d.kPi);

  public static final double fieldWidth = 16.512242;
  public static final double fieldHeight = 8.042264;

  public static final double boundaryWidth = 16.5411912;
  public static final double boundaryHeight = 8.0692752;
}
