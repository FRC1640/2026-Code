package frc.robot.util.autoalign.system.controller;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.autoalign.system.eventshandler.AlignSystemEventBus;
import frc.robot.util.autoalign.system.pointprovider.IAlignPointProvider;
import lombok.Getter;

public abstract class IAlignController {
  @Getter
  protected IAlignPointProvider pointProvider;
  public IAlignController(IAlignPointProvider pp) {
    this.pointProvider = pp;
    pp.attachAlignController(this);
  }
  public abstract ChassisSpeeds calculate(ChassisSpeeds robotChassisSpeeds);
  public abstract boolean isAligning();
  public abstract AlignSystemEventBus getAlignSystemEventBus();
}
