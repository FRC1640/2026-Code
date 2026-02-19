package frc.robot.util.robotswitcher;

import java.util.ArrayList;
import java.util.List;

import frc.robot.sensors.apriltag.CameraConstant;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;

/**
 * Add more robots by adding to this enum
 */
public class RobotType {

  List<SubsystemInfo> enabled = new ArrayList<>();
  String robotName;
  ArrayList<CameraConstant> cameras = new ArrayList<>();
  public RobotType(String robotName, SubsystemInfo... subsystems) {
    this.robotName = robotName;
    for (SubsystemInfo sInfo : subsystems) {
      enabled.add(sInfo);
    }
  }

  public boolean isEnabled(SubsystemInfo subsysInfo) {
    return enabled.contains(subsysInfo);
  }

  public String getName() {
    return robotName;
  }
  public RobotType addAprilTagCamera(CameraConstant cam) {
    cameras.add(cam);
    return this;
  }

  public List<CameraConstant> getCameras() {
    return cameras;
  }
}
