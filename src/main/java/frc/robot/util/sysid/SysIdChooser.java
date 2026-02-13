package frc.robot.util.sysid;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;

public class SysIdChooser {

  private final DriveSubsystem driveSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;
  private final TurretSubsystem turretSubsystem;

  private CommandXboxController controller;

  private static SendableChooser<Command> sysIdChooser = new SendableChooser<Command>();

  public SysIdChooser(DriveSubsystem driveSubsystem, FlywheelSubsystem flywheelSubsystem,
      TurretSubsystem turretSubsystem, CommandXboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.flywheelSubsystem = flywheelSubsystem;
    this.turretSubsystem = turretSubsystem;

    this.controller = controller;
    sysIdInit();
  }

  public void sysIdInit() {
    BooleanSupplier startNext = controller.b();
    BooleanSupplier cancel = controller.a();

    sysIdChooser.addOption("Swerve SysId", CreateSysIdCommand.createCommand(driveSubsystem::sysIdQuasistatic,
        driveSubsystem::sysIdDynamic, "Swerve", startNext, cancel, () -> driveSubsystem.stop()));

    sysIdChooser.addOption("Turret SysId", CreateSysIdCommand.createCommand(turretSubsystem::sysIdQuasistatic,
        turretSubsystem::sysIdDynamic, "Turret", startNext, cancel, () -> turretSubsystem.stop()));

    sysIdChooser.addOption("Flywheel SysId",
        CreateSysIdCommand.createCommand(flywheelSubsystem::sysIdQuasistatic, flywheelSubsystem::sysIdDynamic,
            "Flywheel", startNext, cancel,
            () -> CommandScheduler.getInstance().schedule(flywheelSubsystem.stopCommand())));

    sysIdChooser.setDefaultOption("No SysId Selected", new WaitCommand(0.01));

    // TODO: add more sysId routines here

    SmartDashboard.putData("SysId Routines", sysIdChooser);
  }

  public static Command getSysIdCommand() {
    return sysIdChooser.getSelected();
  }
}
