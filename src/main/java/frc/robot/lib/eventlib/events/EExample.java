package frc.robot.lib.eventlib.events;

import frc.robot.lib.eventlib.IEvent;

/**
 * Example Event Class
 */
public class EExample implements IEvent {
  String eventMessage;
  public EExample(String eventMessage) {
    this.eventMessage = eventMessage;
  }
  public String getMessage() {
    return eventMessage;
  }
}
