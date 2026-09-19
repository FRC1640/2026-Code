package frc.robot.subsystems.drive.weights;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.autoalign.controller.IAlignController;
import frc.robot.util.autoalign.pointprovider.IAlignPointProvider;

public class DriveAlignSysWeight implements DriveWeight {
    IAlignController controller;
    IAlignPointProvider provider;

    DriveSubsystem driveSubsystem;
    public DriveAlignSysWeight(IAlignController controller, IAlignPointProvider provider, DriveSubsystem driveSubsystem) {
        this.controller = controller;
        this.provider = provider;
        this.driveSubsystem = driveSubsystem;
    }
    @Override
    public ChassisSpeeds getSpeeds() {
        return controller.calculate(provider, driveSubsystem.getChassisSpeeds());
    }
}
