package frc.robot.constants;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class FieldConstants {
  public static AprilTagFieldLayout aprilTagLayout;

  private static FieldConstants instance = new FieldConstants();

  private FieldConstants() {
    try {
      aprilTagLayout = new AprilTagFieldLayout("src/main/deploy/resources/2026-rebuilt-welded.json");
    } catch (IOException e) { System.out.println("IOException initializing apriltag layout!"); }
  }
}
