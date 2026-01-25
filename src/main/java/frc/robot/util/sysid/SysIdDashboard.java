package frc.robot.util.sysid;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drive.DriveSubsystem;

public class SysIdDashboard {

    private DriveSubsystem driveSubsystem;
    private CommandXboxController controller;

    private static SendableChooser<Command> sysIdChooser = new SendableChooser<Command>();

    public SysIdDashboard(DriveSubsystem driveSubsystem, CommandXboxController controller) {
        this.driveSubsystem = driveSubsystem;
        this.controller = controller;
        sysIdInit();
    }

    public void sysIdInit() {
        BooleanSupplier startNext = controller.b();
        BooleanSupplier cancel = controller.a();

        sysIdChooser = new SendableChooser<Command>();

        sysIdChooser.addOption("Swerve SysId",
                CreateSysIdCommand.createCommand(
                        driveSubsystem::sysIdQuasistatic,
                        driveSubsystem::sysIdDynamic,
                        "Swerve",
                        startNext,
                        cancel,
                        () -> driveSubsystem.stop()));

        // TODO: add more sysId routines here

        SmartDashboard.putData("SysId Routines", sysIdChooser);
    }

    public static Command getSysIdCommand() {
        return sysIdChooser.getSelected();
    }
}
