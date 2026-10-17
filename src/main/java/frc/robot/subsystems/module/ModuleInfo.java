package frc.robot.subsystems.module;

import frc.robot.subsystems.drive.DriveConstants.PivotId;
import frc.robot.util.control.FeedForwardConstants;

// class to store constants for a module
public class ModuleInfo {

  PivotId id;
  int driveID;
  int steerID;
  int resolverChannel;
  double angleOffset;
  FeedForwardConstants steerFF;

  public ModuleInfo(PivotId id, int driveID, int steerID, int resolverChannel, double angleOffset,
      FeedForwardConstants steerFF) {
    this.id = id;
    this.driveID = driveID;
    this.steerID = steerID;
    this.resolverChannel = resolverChannel;
    this.angleOffset = angleOffset;
    this.steerFF = steerFF;
  }
}
