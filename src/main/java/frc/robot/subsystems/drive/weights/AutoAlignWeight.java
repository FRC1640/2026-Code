package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.autoalign.system.controller.AlignControllerBase;

public class AutoAlignWeight implements DriveWeight {
  AlignControllerBase controller;

  DriveSubsystem driveSubsystem;
  public AutoAlignWeight(AlignControllerBase controller, DriveSubsystem driveSubsystem) {
    this.controller = controller;
    this.driveSubsystem = driveSubsystem;
  }
  @Override
  public ChassisSpeeds getSpeeds() {
    return controller.calculate(driveSubsystem.getChassisSpeeds());
  }
}
