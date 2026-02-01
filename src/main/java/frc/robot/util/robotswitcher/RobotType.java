package frc.robot.util.robotswitcher;

import java.util.ArrayList;
import java.util.List;

import frc.robot.util.wrapper.subsystem.SubsystemInfo;

/**
 * Add more robots by adding to this enum
 */
public class RobotType {

  List<SubsystemInfo> enabled = new ArrayList<>();
  String robotName;

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
}
