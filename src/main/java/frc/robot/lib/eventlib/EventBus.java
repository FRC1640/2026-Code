package frc.robot.lib.eventlib;

import java.util.HashMap;
import java.util.Map;

/**
 * Event Bus Class. Manages events that come in and out. Stolen von my defunct
 * library
 */
public class EventBus {
  protected final Map<Class<? extends IEvent>, EventHandler<? extends IEvent>> events;

  public EventBus() {
    this.events = new HashMap<>();
  }

  /**
   * Fires an event
   *
   * @param event
   *            Event
   */
  @SuppressWarnings("unchecked")
  public <E extends IEvent> void fireEvent(E event) {
    EventHandler<E> handler = (EventHandler<E>) getHandler(event.getClass());

    if (handler == null) {
      addEvent(new EventHandler<>(event.getClass()));
    }
    handler = (EventHandler<E>) getHandler(event.getClass());
    handler.fire(event);
  }

  /**
   * Registers a Listener for Event Can be used as a decorator
   *
   * @implNote if the instance calling this method is not permanent, it is highly
   *           recommended to unregister the listener when the class is
   *           destroyed/closed. Otherwise, you could run into a memory leak.
   * @param eventClass
   *            Event Class that you are listening for
   * @param listener
   *            Listener Handler
   */
  public <E extends IEvent> EventBus registerListener(Class<E> eventClass, IEventListener<E> listener) {
    EventHandler<E> handler = getHandler(eventClass);

    if (handler == null) {
      addEvent(new EventHandler<>(eventClass));
    }
    handler = getHandler(eventClass);
    handler.add(listener);
    return this;
  }

  /**
   * Unregisters a Listener Can be used as a decorator
   *
   * @param eventClass
   *            Event Class that you are listening for
   * @param listener
   *            Listener Handler
   */
  public <E extends IEvent> EventBus unregisterListener(Class<E> eventClass, IEventListener<E> listener) {
    EventHandler<E> handler = getHandler(eventClass);

    if (handler == null) {
      return this;
    }
    handler = getHandler(eventClass);
    handler.remove(listener);
    return this;
  }

  private <E extends IEvent> EventBus addEvent(EventHandler<E> eventHandler) {
    events.put(eventHandler.getEventClass(), eventHandler);
    return this;
  }

  /**
   * Removes listeners from an event
   *
   * @param eventClass
   *            Event Class Can be used as a decorator
   */
  public <E extends IEvent> EventBus removeEventListeners(Class<E> eventClass) {
    events.remove(eventClass);
    return this;
  }

  /**
   * Clears all event listeners on bus Can be used as a decorator
   */
  public EventBus clearEventsListeners() {
    events.clear();
    return this;
  }

  @SuppressWarnings("unchecked")
  private <E extends IEvent> EventHandler<E> getHandler(Class<E> eventClass) {
    return (EventHandler<E>) events.get(eventClass);
  }
}
