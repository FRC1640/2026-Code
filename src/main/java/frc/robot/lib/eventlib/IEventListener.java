package frc.robot.lib.eventlib;

/**
 * Event Listener Functionality Class.
 */
@FunctionalInterface
public interface IEventListener<E extends IEvent> {
  void execute(E event);
}
