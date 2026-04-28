package frc.robot.util.sysid;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class SysIdChooser {

  private final DriveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;

  private CommandXboxController controller;

  private static SendableChooser<Command> sysIdChooser = new SendableChooser<Command>();

  public SysIdChooser(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem, CommandXboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;

    this.controller = controller;
    sysIdInit();
  }

  public void sysIdInit() {
    BooleanSupplier startNext = controller.a();
    BooleanSupplier cancel = controller.b();

    sysIdChooser.addOption("Swerve SysId",
        CreateSysIdCommand.createCommand(driveSubsystem::sysIdQuasistatic, driveSubsystem::sysIdDynamic,
            "Swerve", startNext, cancel,
            () -> CommandScheduler.getInstance().schedule(driveSubsystem.stopCommand())));

    sysIdChooser.addOption("Turret SysId",
        CreateSysIdCommand.createCommand(turretSubsystem::sysIdQuasistatic, turretSubsystem::sysIdDynamic,
            "Turret", startNext, cancel,
            () -> CommandScheduler.getInstance().schedule(turretSubsystem.stopCommand())));

    sysIdChooser.addOption("Shooter SysId",
        CreateSysIdCommand.createCommand(shooterSubsystem::sysIdQuasistatic, shooterSubsystem::sysIdDynamic,
            "Shooter", startNext, cancel,
            () -> CommandScheduler.getInstance().schedule(shooterSubsystem.stopCommand())));

    sysIdChooser.setDefaultOption("No SysId Selected", new WaitCommand(0.01));

    // add more sysId routines here

    SmartDashboard.putData("SysId Routines", sysIdChooser);
  }

  public static Command getSysIdCommand() {
    return sysIdChooser.getSelected();
  }
}
