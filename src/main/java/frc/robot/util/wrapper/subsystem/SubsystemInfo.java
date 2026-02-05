package frc.robot.util.wrapper.subsystem;

/**
 * Add more info for more subsystems when deemed neccessary
 */
public class SubsystemInfo {

  String subsystemName;
  // add more to this if you want to add more info

  public SubsystemInfo(String name) {
    this.subsystemName = name;
  }

  public String getName() {
    return subsystemName;
  }
}
