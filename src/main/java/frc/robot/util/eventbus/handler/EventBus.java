package frc.robot.util.eventbus.handler;

import java.util.HashMap;
import java.util.Map;

import frc.robot.util.eventbus.events.IEvent;
import frc.robot.util.eventbus.events.IEventListener;

public class EventBus {

    protected final Map<Class<? extends IEvent>, EventHandler<? extends IEvent>> events;

    public EventBus() {
        this.events = new HashMap<>();
    }

    public <E extends IEvent> EventBus addEvent(EventHandler<E> eventHandler) {
        events.put(eventHandler.getEventClass(), eventHandler);
        return this;
    }

    public <E extends IEvent> EventBus removeEvent(Class<E> eventClass) {
        events.remove(eventClass);
        return this;
    }

    public EventBus clearEvents() {
        events.clear();
        return this;
    }

    public <E extends IEvent> void fireEvent(Class<E> eventClass, E event) {
        EventHandler<E> handler = getHandler(eventClass);

        if (handler == null) {
            throw new IllegalArgumentException(
                    "Event is not registered: " + eventClass.getName()
            );
        }

        handler.fire(event);
    }

    public <E extends IEvent> void registerListener(
            Class<E> eventClass,
            IEventListener<E> listener
    ) {
        EventHandler<E> handler = getHandler(eventClass);

        if (handler == null) {
            throw new IllegalArgumentException(
                    "Event is not registered: " + eventClass.getName()
            );
        }

        handler.add(listener);
    }

    @SuppressWarnings("unchecked")
    private <E extends IEvent> EventHandler<E> getHandler(Class<E> eventClass) {
        return (EventHandler<E>) events.get(eventClass);
    }
}