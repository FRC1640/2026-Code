package frc.robot.util.autoalign.system.eventshandler;

import frc.robot.util.autoalign.system.eventshandler.events.ECompleteReachPoint;
import frc.robot.util.eventbus.handler.EventBus;
import frc.robot.util.eventbus.handler.EventHandler;

public class AlignSystemEventBus extends EventBus {
     public AlignSystemEventBus() {
        super();
        addEvent(new EventHandler<>(ECompleteReachPoint.class));
    }
}
