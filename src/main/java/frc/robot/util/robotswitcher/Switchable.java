package frc.robot.util.robotswitcher;

import java.util.HashMap;
import java.util.Map;

import frc.robot.constants.RobotConstants.RobotInformation;

public class Switchable<T> {
  Map<String, T> values = new HashMap<>();
  T defaultValue;

  public Switchable(T defaultValue) {
    this.defaultValue = defaultValue;
  }

  public Switchable<T> addAlt(RobotType robot, T value) {
    values.put(robot.getName(), value);
    return this;
  }

  public T get() {
    return values.getOrDefault(RobotInformation.robot.getName(), defaultValue);
  }

  public static <T> Switchable<T> of(T value) {
    return new Switchable<T>(value);
  }
}
