package frc.robot.util.sysid;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.drive.DriveSubsystem;

public class SysIdManager {

    private DriveSubsystem driveSubsystem;
    private CommandXboxController controller;

    private static ArrayList<Command> sysIdCommands = new ArrayList<Command>(3);

    private static byte selectedRoutineType;

    public SysIdManager(DriveSubsystem driveSubsystem, CommandXboxController controller) {
        this.driveSubsystem = driveSubsystem;
        this.controller = controller;
    }

    public void sysIdInit() {
        selectedRoutineType = 0;
        BooleanSupplier startNext = controller.b();
        BooleanSupplier cancel = controller.a();
        BooleanSupplier nextRoutine = controller.y();

        new Trigger(nextRoutine).onTrue(new InstantCommand(() -> selectedRoutineType++));

        Logger.getLogger("SysIdManager").info("Selected SysId Routine: " + selectedRoutineType);

        sysIdCommands.add(
            CreateSysIdCommand.createCommand(
                driveSubsystem::sysIdQuasistatic,
                driveSubsystem::sysIdDynamic,
                "Swerve",
                startNext,
                cancel,
                () -> driveSubsystem.stop()));

        // TODO: add more sysId routines here
    }

    public static Command getSysIdCommand() {
        try {
            Logger.getLogger("SysIdManager").info("Selected SysId Routine: " + selectedRoutineType);
            return sysIdCommands.get(selectedRoutineType);
        } catch (IndexOutOfBoundsException e) {
            selectedRoutineType = 0;
            Logger.getLogger("SysIdManager").warning("Selected SysId Routine out of bounds, defaulting to 0");
            Logger.getLogger("SysIdManager").info("Selected SysId Routine: " + selectedRoutineType);
            return sysIdCommands.get(selectedRoutineType);
        }
    }
}
