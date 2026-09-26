package frc.robot.util.autoalign.system.controller;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.autoalign.system.events.AlignSystemEventBus;
import frc.robot.util.autoalign.system.pointprovider.IAlignPointProvider;

public interface IAlignController {
  ChassisSpeeds calculate(ChassisSpeeds robotChassisSpeeds);
  boolean isAligning();
  IAlignPointProvider getPointProvider();
  AlignSystemEventBus getAlignSystemEventBus();
}
