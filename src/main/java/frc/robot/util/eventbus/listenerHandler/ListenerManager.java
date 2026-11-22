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
  private final Map<BooleanSupplier, List<IEvent>> fireEventHashMap;
  private final List<BooleanSupplier> awaitingToggleBack;
  private final EventBus eventBus;

  public ListenerManager(EventBus eventBus) {
    this.eventBus = eventBus;
    this.fireEventHashMap = new HashMap<>();
    this.awaitingToggleBack = new ArrayList<>();
  }

  public void registerEventTrigger(BooleanSupplier condition, IEvent fireEvent) {
    fireEventHashMap.computeIfAbsent(condition, (k) -> new ArrayList<>()).add(fireEvent);

  }

  @Override
  public void periodic() {
    for (BooleanSupplier bs : fireEventHashMap.keySet()) {
      boolean output = bs.getAsBoolean();
      boolean inToggleBack = awaitingToggleBack.contains(bs);
      if (output && !inToggleBack) {
        for (IEvent event : fireEventHashMap.get(bs)) {
          eventBus.fireEvent(event);
        }
        awaitingToggleBack.add(bs);
      } else if (!output && inToggleBack) {
        awaitingToggleBack.remove(bs);
      }
    }
  }
}
