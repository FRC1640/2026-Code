package frc.robot.util.autoalign.controller;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.autoalign.pointprovider.IAlignPointProvider;

public interface IAlignController {
  ChassisSpeeds calculate(IAlignPointProvider pointProvider, ChassisSpeeds robotChassisSpeeds);
  boolean isAligning();
}
