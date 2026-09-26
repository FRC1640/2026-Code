package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.autoalign.system.controller.IAlignController;

public class DriveAlignSysWeight implements DriveWeight {
  IAlignController controller;

  DriveSubsystem driveSubsystem;
  public DriveAlignSysWeight(IAlignController controller, DriveSubsystem driveSubsystem) {
    this.controller = controller;
    this.driveSubsystem = driveSubsystem;
  }
  @Override
  public ChassisSpeeds getSpeeds() {
    return controller.calculate(driveSubsystem.getChassisSpeeds());
  }
}
