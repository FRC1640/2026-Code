package frc.robot.util.sysid;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drive.DriveSubsystem;

import frc.robot.util.networktables.SysIdChooser;

public class SysIdManager {

    private DriveSubsystem driveSubsystem;
    private CommandXboxController controller;

    private static ArrayList<Command> sysIdCommands = new ArrayList<Command>(3);

    private static SysIdChooser sysIdChooser = new SysIdChooser("SysId Routines", sysIdCommands);

    public SysIdManager(DriveSubsystem driveSubsystem, CommandXboxController controller) {
        this.driveSubsystem = driveSubsystem;
        this.controller = controller;
    }

    public void sysIdInit() {
        BooleanSupplier startNext = controller.b();
        BooleanSupplier cancel = controller.a();

        // sysIdCommands.add(
        //     CreateSysIdCommand.createCommand(
        //         driveSubsystem::sysIdQuasistatic,
        //         driveSubsystem::sysIdDynamic,
        //         "Swerve",
        //         startNext,
        //         cancel,
        //         () -> driveSubsystem.stop()));

        sysIdCommands.add(new InstantCommand(() -> System.out.println("SysId Routine Started")).withName("SysId Start Command"));

        sysIdChooser = new SysIdChooser("SysId Routines", sysIdCommands);

        // TODO: add more sysId routines here
    }

    public static Command getSysIdCommand() {
        return sysIdChooser.getSysIdRoutine();
    }
}
