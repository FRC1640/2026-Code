package frc.robot.util.eventbus.events;

@FunctionalInterface
public interface IEventListener<E extends IEvent> {
    void execute(E event);
}