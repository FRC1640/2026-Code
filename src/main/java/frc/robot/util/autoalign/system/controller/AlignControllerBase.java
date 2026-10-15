package frc.robot.util.autoalign.system.controller;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.autoalign.system.eventshandler.AlignSystemEventBus;
import frc.robot.util.autoalign.system.pointprovider.IAlignPointProvider;
import frc.robot.util.eventbus.listenerHandler.ListenerManager;
import lombok.Getter;

public abstract class AlignControllerBase {
  @Getter
  protected IAlignPointProvider pointProvider;
  @Getter
  protected AlignSystemEventBus alignSystemEventBus;
  @Getter
  protected ListenerManager listenerManager;
  public AlignControllerBase(IAlignPointProvider pp) {
    this.pointProvider = pp;
    this.alignSystemEventBus = new AlignSystemEventBus();
    listenerManager = new ListenerManager(alignSystemEventBus);
    pp.attachAlignController(this);
  }
  public abstract ChassisSpeeds calculate(ChassisSpeeds robotChassisSpeeds);
  public abstract boolean isAligning();
}
