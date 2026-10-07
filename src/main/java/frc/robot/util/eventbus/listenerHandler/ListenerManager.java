package frc.robot.util.eventbus.listenerHandler;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

import frc.robot.util.periodic.PeriodicBase;
import frc.robot.util.eventbus.events.IEvent;
import frc.robot.util.eventbus.handler.EventBus;

public class ListenerManager extends PeriodicBase {
  Map<BooleanSupplier, List<IEvent>> fireEventHashMap;
  List<BooleanSupplier> awaitingToggleBack;
  EventBus eventBus;
  public ListenerManager(EventBus eventBus) {
    this.eventBus = eventBus;
    this.fireEventHashMap = new HashMap<>();
    this.awaitingToggleBack = new ArrayList<>();
  }
  public void registerEventTrigger(BooleanSupplier condition, IEvent fireEvent) {
    List<IEvent> ev = fireEventHashMap.getOrDefault(condition, new ArrayList<>());
    ev.add(fireEvent);
    fireEventHashMap.put(condition, ev);
  }
  @Override
  public void periodic() {
    awaitingToggleBack.forEach((x) -> {
      if (!x.getAsBoolean()) {
        awaitingToggleBack.remove(x);
      }
    });
    fireEventHashMap.keySet().forEach((x) -> {
      if (x.getAsBoolean() && !awaitingToggleBack.contains(x)) {
        for (IEvent event : fireEventHashMap.get(x)) {
          eventBus.fireEvent(event);
        }
      }
      awaitingToggleBack.add(x);
      return;
    });
  }
}
