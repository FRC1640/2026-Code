package frc.robot.util.eventbus;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.Test;

import frc.robot.lib.eventlib.EventBus;
import frc.robot.lib.eventlib.TriggerEventManager;
import frc.robot.lib.eventlib.events.EExample;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class EventBusSystemTest {
  @Test
  public void testEventBus() {
    AtomicReference<String> valueToCheck = new AtomicReference<>();
    EventBus eventBus = new EventBus().registerListener(EExample.class, event -> {
      valueToCheck.set(event.getMessage());
    });
    eventBus.fireEvent(new EExample("i have a message!"));
    assertEquals("i have a message!", valueToCheck.get());
  }
  @Test
  public void testTriggerManager() {
    AtomicBoolean condition = new AtomicBoolean(false);
    AtomicInteger number = new AtomicInteger(1);
    EventBus eventBus = new EventBus();
    TriggerEventManager triggerEventManager = new TriggerEventManager(eventBus);
    triggerEventManager.registerEventTrigger(condition::get, () -> new EExample("It triggered! " + number.get())); // this
    // line
    // is
    // triggered
    // first,
    // and
    // the
    // listener
    // responds
    // &
    // increments.
    // my
    // mistake
    AtomicReference<String> valueToCheck = new AtomicReference<>();
    eventBus.registerListener(EExample.class, event -> {
      number.incrementAndGet();
      valueToCheck.set(event.getMessage());
    });
    condition.set(true);
    triggerEventManager.updatePeriodic();
    assertEquals("It triggered! 1", valueToCheck.get());
    condition.set(false);
    triggerEventManager.updatePeriodic();
    condition.set(true);
    triggerEventManager.updatePeriodic();
    assertEquals("It triggered! 2", valueToCheck.get());
  }
}
