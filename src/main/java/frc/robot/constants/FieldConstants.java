package frc.robot.constants;

import java.io.IOException;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;

public class FieldConstants {
  public static AprilTagFieldLayout aprilTagLayout;

  private static FieldConstants instance = new FieldConstants();

  private FieldConstants() {
    try {
      aprilTagLayout = new AprilTagFieldLayout(
          Filesystem.getDeployDirectory() + "/resources/2026-rebuilt-welded.json");
    } catch (IOException e) {
      System.out.println("IOException initializing apriltag layout!");
    }
  }

  public static final VisionSystemSim visionSim = new VisionSystemSim("main");

  public static final VisionSystemSim getVisionSim() {
    return visionSim;
  }

  public static final double height = 8.07;
  public static final double width = 16.54;
}
