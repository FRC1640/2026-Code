package frc.robot.util.eventbus.handler;

import java.util.HashMap;
import java.util.Map;
import java.util.logging.LogManager;
import java.util.logging.Logger;

import frc.robot.util.eventbus.events.IEvent;
import frc.robot.util.eventbus.events.IEventListener;

public class EventBus {
  public static Logger EVENT_LOGGER = LogManager.getLogManager().getLogger("[EVENTBUS]");
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
  @SuppressWarnings("unchecked")
  public <E extends IEvent> void fireEvent(E event) {
    EventHandler<E> handler = (EventHandler<E>) getHandler(event.getClass());

    if (handler == null) {
      addEvent(new EventHandler<>(event.getClass()));
      EVENT_LOGGER.warning("Event " + event.getClass()
          + " added to bus because of an attempt to fire the event. Please register this event if you want to use it. ");
    }
    handler = (EventHandler<E>) getHandler(event.getClass());
    handler.fire(event);
  }

  public <E extends IEvent> void registerListener(Class<E> eventClass, IEventListener<E> listener) {
    EventHandler<E> handler = getHandler(eventClass);

    if (handler == null) {
      addEvent(new EventHandler<>(eventClass));
      EVENT_LOGGER.warning("Event " + eventClass.getCanonicalName()
          + " added to bus because of an attempt to register a listener to the event. Please register this event if you want to use it. ");
    }
    handler = getHandler(eventClass);
    handler.add(listener);
  }

  @SuppressWarnings("unchecked")
  private <E extends IEvent> EventHandler<E> getHandler(Class<E> eventClass) {
    return (EventHandler<E>) events.get(eventClass);
  }
}
