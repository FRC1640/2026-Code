package frc.robot.lib.eventlib;

import java.util.*;
import java.util.function.Supplier;

import frc.robot.util.periodic.PeriodicBase;

/**
 * Trigger system for events. Fires events when a BooleanSupplier's condition(s)
 * are met.
 */
public class TriggerEventManager extends PeriodicBase {
  private final Map<Supplier<Boolean>, List<Supplier<IEvent>>> fireEventHashMap;
  private final List<Supplier<Boolean>> awaitingToggleBack;
  private final EventBus eventBus;

  public TriggerEventManager(EventBus eventBus) {
    Objects.requireNonNull(eventBus);
    this.eventBus = eventBus;
    this.fireEventHashMap = new HashMap<>();
    this.awaitingToggleBack = new ArrayList<>();
  }

  public void registerEventTrigger(Supplier<Boolean> condition, Supplier<IEvent> fireEvent) {
    fireEventHashMap.computeIfAbsent(condition, (k) -> new ArrayList<>()).add(fireEvent);

  }

  public void updatePeriodic() {
    for (Supplier<Boolean> bs : fireEventHashMap.keySet()) {
      boolean output = bs.get();
      boolean inToggleBack = awaitingToggleBack.contains(bs);
      if (output && !inToggleBack) {
        for (IEvent event : fireEventHashMap.get(bs).stream().map(Supplier::get).toList()) {
          eventBus.fireEvent(event);
        }
        awaitingToggleBack.add(bs);
      } else if (!output && inToggleBack) {
        awaitingToggleBack.remove(bs);
      }
    }
  }

  @Override
  public void periodic() {
    updatePeriodic();
  }
}
