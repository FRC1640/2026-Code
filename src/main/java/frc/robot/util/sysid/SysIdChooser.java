package frc.robot.util.sysid;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drive.DriveSubsystem;

public class SysIdChooser {

    private DriveSubsystem driveSubsystem;
    private CommandXboxController controller;

    private static SendableChooser<Command> sysIdCommandChooser = new SendableChooser<Command>();

    public SysIdChooser(DriveSubsystem driveSubsystem, CommandXboxController controller) {
        this.driveSubsystem = driveSubsystem;
        this.controller = controller;
        sysIdInit();
    }

    public void sysIdInit() {
        BooleanSupplier startNext = controller.b();
        BooleanSupplier cancel = controller.a();

        sysIdCommandChooser = new SendableChooser<Command>();

        sysIdCommandChooser.addOption("Swerve SysId",
                CreateSysIdCommand.createCommand(
                        driveSubsystem::sysIdQuasistatic,
                        driveSubsystem::sysIdDynamic,
                        "Swerve",
                        startNext,
                        cancel,
                        () -> driveSubsystem.stop()));

        // TODO: add more sysId routines here

        SmartDashboard.putData("SysId Routines", sysIdCommandChooser);
    }

    public static Command getSysIdCommand() {
        return sysIdCommandChooser.getSelected();
    }
}
