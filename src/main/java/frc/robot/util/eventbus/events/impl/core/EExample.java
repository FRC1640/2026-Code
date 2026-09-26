package frc.robot.util.eventbus.events.impl.core;

import frc.robot.util.eventbus.events.IEvent;
// Example Event
public class EExample implements IEvent {
    String eventMessage;
    public EExample(String eventMessage) {
        this.eventMessage = eventMessage;
    }
    String getMessage() {
        return eventMessage;
    }
}
